#include "stepper_motor.h"
#include "../app.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(stepper_motor, LOG_LEVEL_INF);

namespace component {

static constexpr uint64_t NS_PER_MIN = 60000000000ULL;

uint64_t StepperMotor::computeInterval(uint32_t rpm) const
{
    return NS_PER_MIN / ((uint64_t)rpm * cfg_->steps_per_rev);
}

int StepperMotor::physicalDir(MotionDir logical) const
{
    int d = (logical == MotionDir::POS) ? 1 : -1;
    return cfg_->invert_dir ? -d : d;
}

bool StepperMotor::checkLimits(int32_t target_pos) const
{
    if (!cfg_->has_limits) {
        return true;
    }
    return target_pos >= cfg_->soft_limit_min &&
           target_pos <= cfg_->soft_limit_max;
}

void StepperMotor::stepperEventCb(const struct device *dev,
                                   enum stepper_ctrl_event event, void *ud)
{
    auto *self = static_cast<StepperMotor *>(ud);
    if (event == STEPPER_CTRL_EVENT_STEPS_COMPLETED ||
        event == STEPPER_CTRL_EVENT_STOPPED) {
        self->onMoveComplete();
    }
}

void StepperMotor::onMoveComplete()
{
    if (pending_steps_ != 0) {
        position_ += pending_steps_;
        pending_steps_ = 0;
    }
    moving_ = false;
    cur_dir_ = MotionDir::NONE;
    k_sem_give(&done_sem_);
}

void StepperMotor::onAlmAsserted()
{
    alarm_ = true;
    if (alarm_cb_) {
        alarm_cb_(*this, alarm_ctx_);
    }
}

int StepperMotor::init(const StepperMotorConfig &cfg)
{
    cfg_ = &cfg;
    position_ = 0;
    homed_ = false;
    moving_ = false;
    alarm_ = false;
    enabled_ = false;
    cur_dir_ = MotionDir::NONE;
    pending_steps_ = 0;

    k_sem_init(&done_sem_, 0, 1);

    if (!device_is_ready(cfg_->stepper_dev)) {
        LOG_ERR("stepper device not ready");
        return -ENODEV;
    }

    stepper_ctrl_set_event_cb(cfg_->stepper_dev, stepperEventCb, this);

    if (cfg_->ena_pin && gpio_is_ready_dt(cfg_->ena_pin)) {
        gpio_pin_configure_dt(cfg_->ena_pin, GPIO_OUTPUT_INACTIVE);
    }

    for (uint8_t i = 0; i < cfg_->alm_count; i++) {
        if (gpio_is_ready_dt(&cfg_->alm_pins[i])) {
            gpio_pin_configure_dt(&cfg_->alm_pins[i], GPIO_INPUT);
        }
    }

    for (uint8_t i = 0; i < cfg_->pend_count; i++) {
        if (gpio_is_ready_dt(&cfg_->pend_pins[i])) {
            gpio_pin_configure_dt(&cfg_->pend_pins[i], GPIO_INPUT);
        }
    }

    if (cfg_->has_home && cfg_->home_sensor && gpio_is_ready_dt(cfg_->home_sensor)) {
        gpio_pin_configure_dt(cfg_->home_sensor, GPIO_INPUT);
    }

    LOG_INF("stepper init OK (limits:%s home:%s pend:%d alm:%d)",
            cfg_->has_limits ? "yes" : "no",
            cfg_->has_home ? "yes" : "no",
            cfg_->pend_count,
            cfg_->alm_count);
    return 0;
}

void StepperMotor::setEnabled(bool on)
{
    if (cfg_->ena_pin && gpio_is_ready_dt(cfg_->ena_pin)) {
        gpio_pin_set_dt(cfg_->ena_pin, on ? 1 : 0);
        enabled_ = on;
    }
}

int StepperMotor::home()
{
    if (!cfg_->has_home || !cfg_->home_sensor) {
        return -ENOTSUP;
    }

    if (alarm_) {
        return static_cast<int>(Error::IN_FAULT);
    }

    auto home_phys_dir = (physicalDir(cfg_->home_dir) > 0)
                             ? STEPPER_CTRL_DIRECTION_POSITIVE
                             : STEPPER_CTRL_DIRECTION_NEGATIVE;
    auto away_phys_dir = (physicalDir(cfg_->home_dir) > 0)
                             ? STEPPER_CTRL_DIRECTION_NEGATIVE
                             : STEPPER_CTRL_DIRECTION_POSITIVE;

    int err;

    // Phase 1: if already on sensor, back off first
    if (gpio_pin_get_dt(cfg_->home_sensor)) {
        LOG_INF("home: already on sensor, backing off...");
        err = stepper_ctrl_set_microstep_interval(cfg_->stepper_dev,
                                                   computeInterval(cfg_->home_rpm / 2));
        if (err) return err;

        moving_ = true;
        err = stepper_ctrl_run(cfg_->stepper_dev, away_phys_dir);
        if (err) { moving_ = false; return err; }

        while (gpio_pin_get_dt(cfg_->home_sensor)) {
            if (alarm_) { stepper_ctrl_stop(cfg_->stepper_dev); moving_ = false; return static_cast<int>(Error::IN_FAULT); }
            k_msleep(1);
        }
        stepper_ctrl_stop(cfg_->stepper_dev);
        moving_ = false;
        k_msleep(50);
    }

    // Phase 2: approach sensor at home speed
    LOG_INF("home: approaching sensor...");
    err = stepper_ctrl_set_microstep_interval(cfg_->stepper_dev, computeInterval(cfg_->home_rpm));
    if (err) return err;

    moving_ = true;
    cur_dir_ = cfg_->home_dir;

    err = stepper_ctrl_run(cfg_->stepper_dev, home_phys_dir);
    if (err) { moving_ = false; cur_dir_ = MotionDir::NONE; return err; }

    while (!gpio_pin_get_dt(cfg_->home_sensor)) {
        if (alarm_) { stepper_ctrl_stop(cfg_->stepper_dev); moving_ = false; cur_dir_ = MotionDir::NONE; return static_cast<int>(Error::IN_FAULT); }
        k_msleep(1);
    }
    stepper_ctrl_stop(cfg_->stepper_dev);
    moving_ = false;
    k_msleep(50);

    // Phase 3: overshoot into sensor a bit more (1/4 rev)
    LOG_INF("home: overdriving into sensor...");
    int32_t overshoot = (int32_t)(cfg_->steps_per_rev / 4);
    pending_steps_ = 0;
    err = stepper_ctrl_set_microstep_interval(cfg_->stepper_dev, computeInterval(cfg_->home_rpm / 2));
    if (err) return err;

    k_sem_reset(&done_sem_);
    pending_steps_ = overshoot;
    moving_ = true;

    err = stepper_ctrl_move_by(cfg_->stepper_dev,
             (home_phys_dir == STEPPER_CTRL_DIRECTION_POSITIVE) ? overshoot : -overshoot);
    if (err) { moving_ = false; pending_steps_ = 0; return err; }

    k_sem_take(&done_sem_, K_MSEC(5000));
    moving_ = false;
    pending_steps_ = 0;
    k_msleep(100);

    // Phase 4: back off slowly until sensor opens = home position
    LOG_INF("home: backing off to edge...");
    err = stepper_ctrl_set_microstep_interval(cfg_->stepper_dev, computeInterval(cfg_->home_rpm / 3));
    if (err) return err;

    moving_ = true;
    err = stepper_ctrl_run(cfg_->stepper_dev, away_phys_dir);
    if (err) { moving_ = false; return err; }

    while (gpio_pin_get_dt(cfg_->home_sensor)) {
        if (alarm_) { stepper_ctrl_stop(cfg_->stepper_dev); moving_ = false; return static_cast<int>(Error::IN_FAULT); }
        k_msleep(1);
    }
    stepper_ctrl_stop(cfg_->stepper_dev);
    moving_ = false;
    cur_dir_ = MotionDir::NONE;
    position_ = 0;
    homed_ = true;

    LOG_INF("homed OK (falling edge)");
    return 0;
}

StepperMotor::Error StepperMotor::go(int32_t steps, uint32_t rpm)
{
    if (alarm_) return Error::IN_FAULT;
    if (moving_) return Error::BUSY;
    if (rpm == 0) return Error::NOT_READY;

    if (cfg_->has_limits && homed_ && !app::isFactory()) {
        int32_t target = position_ + steps;
        if (!checkLimits(target)) {
            LOG_WRN("soft limit: pos=%d target=%d min=%d max=%d",
                    position_, target, cfg_->soft_limit_min, cfg_->soft_limit_max);
            return Error::SOFT_LIMIT;
        }
    }

    int err = stepper_ctrl_set_microstep_interval(cfg_->stepper_dev, computeInterval(rpm));
    if (err) return Error::NOT_READY;

    MotionDir logical = (steps >= 0) ? MotionDir::POS : MotionDir::NEG;
    int phys = physicalDir(logical);
    int32_t abs_steps = (steps >= 0) ? steps : -steps;

    k_sem_reset(&done_sem_);
    cur_dir_ = logical;
    moving_ = true;
    pending_steps_ = steps;

    err = stepper_ctrl_move_by(cfg_->stepper_dev, (int32_t)abs_steps * phys);
    if (err) {
        moving_ = false;
        cur_dir_ = MotionDir::NONE;
        pending_steps_ = 0;
        return Error::NOT_READY;
    }

    uint64_t exp_ms = (uint64_t)abs_steps * 60000ULL /
                      ((uint64_t)rpm * cfg_->steps_per_rev);
    k_timeout_t timeout = K_MSEC(exp_ms * 2 + 2000);

    if (k_sem_take(&done_sem_, timeout) != 0) {
        stepper_ctrl_stop(cfg_->stepper_dev);
        moving_ = false;
        cur_dir_ = MotionDir::NONE;
        pending_steps_ = 0;
        LOG_WRN("move timeout");
        return Error::TIMEOUT;
    }

    return Error::OK;
}

StepperMotor::Error StepperMotor::goTo(int32_t target, uint32_t rpm)
{
    if (!homed_ && cfg_->has_home) return Error::NOT_HOMED;
    int32_t delta = target - position_;
    if (delta == 0) return Error::OK;
    return go(delta, rpm);
}

StepperMotor::Error StepperMotor::run(uint32_t rpm, MotionDir dir)
{
    if (alarm_) return Error::IN_FAULT;
    if (moving_) return Error::BUSY;
    if (rpm == 0) return Error::NOT_READY;

    int err = stepper_ctrl_set_microstep_interval(cfg_->stepper_dev, computeInterval(rpm));
    if (err) return Error::NOT_READY;

    auto ctrl_dir = (physicalDir(dir) > 0)
                        ? STEPPER_CTRL_DIRECTION_POSITIVE
                        : STEPPER_CTRL_DIRECTION_NEGATIVE;
    cur_dir_ = dir;
    moving_ = true;

    err = stepper_ctrl_run(cfg_->stepper_dev, ctrl_dir);
    if (err) {
        moving_ = false;
        cur_dir_ = MotionDir::NONE;
        return Error::NOT_READY;
    }

    return Error::OK;
}

void StepperMotor::stop()
{
    stepper_ctrl_stop(cfg_->stepper_dev);
    moving_ = false;
    cur_dir_ = MotionDir::NONE;
}

void StepperMotor::emergencyStop()
{
    stepper_ctrl_stop(cfg_->stepper_dev);
    moving_ = false;
    cur_dir_ = MotionDir::NONE;
}

} // namespace component
