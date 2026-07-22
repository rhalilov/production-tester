#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>

namespace component {

enum class MotionDir : int8_t { NEG = -1, NONE = 0, POS = 1 };

enum class AlmPolicy : uint8_t {
    ANY_TRIGGERS_FAULT,
    ALL_REQUIRED,
};

enum class PendPolicy : uint8_t {
    ALL_REQUIRED,
    ANY_CONFIRMS,
};

struct AccelProfile {
    uint32_t start_rpm;     // ramp start speed (>0)
    uint32_t accel_rpm_s;   // acceleration in rpm/s (0 = no ramp, instant speed)
};

struct StepperMotorConfig {
    const struct device *stepper_dev;

    const gpio_dt_spec *ena_pin;

    const gpio_dt_spec *alm_pins;
    uint8_t alm_count;
    AlmPolicy alm_policy;

    const gpio_dt_spec *pend_pins;
    uint8_t pend_count;
    PendPolicy pend_policy;

    bool has_home;
    const gpio_dt_spec *home_sensor;
    MotionDir home_dir;
    bool auto_home;
    uint32_t home_rpm;

    bool has_limits;
    int32_t soft_limit_min;    // steps from home
    int32_t soft_limit_max;
    int32_t safe_position;

    bool persist_position;
    bool invert_dir;

    AccelProfile accel;
    uint32_t steps_per_rev;
    float mm_per_rev;       // 0 = no linear conversion available
};

class StepperMotor {
public:
    using AlarmCallback = void(*)(StepperMotor &motor, void *context);

    enum class Error {
        OK = 0,
        NOT_READY = -1,
        SOFT_LIMIT = -2,
        IN_FAULT = -3,
        NOT_HOMED = -4,
        TIMEOUT = -5,
        BUSY = -6,
    };

    StepperMotor() : cfg_(nullptr), position_(0), homed_(false),
                     moving_(false), alarm_(false), enabled_(false),
                     alarm_cb_(nullptr), alarm_ctx_(nullptr) {}

    int init(const StepperMotorConfig &cfg);
    int home();

    // Blocking moves
    Error go(int32_t steps, uint32_t rpm);
    Error goTo(int32_t position, uint32_t rpm);

    // Non-blocking move initiation (pair with waitDone() or poll isMoving())
    Error startGo(int32_t steps, uint32_t rpm);
    Error startGoTo(int32_t position, uint32_t rpm);
    void abortMove();

    // Non-blocking continuous run
    Error run(uint32_t rpm, MotionDir dir);
    void stop();
    void emergencyStop();

    // Enable/disable motor driver
    void setEnabled(bool on);
    bool isEnabled() const { return enabled_; }

    // State
    bool isMoving() const { return moving_; }
    int32_t position() const { return position_; }
    bool isHomed() const { return homed_; }
    bool inAlarm() const { return alarm_; }
    MotionDir currentDir() const { return cur_dir_; }

    void setAlarmCallback(AlarmCallback cb, void *ctx) {
        alarm_cb_ = cb;
        alarm_ctx_ = ctx;
    }

    void clearAlarm() { alarm_ = false; }

    // Called from ISR/poll context
    void onAlmAsserted();
    void onMoveComplete();

    const StepperMotorConfig &config() const { return *cfg_; }

    int32_t mmToSteps(float mm) const {
        if (!cfg_ || cfg_->mm_per_rev <= 0.0f) return 0;
        return (int32_t)(mm * cfg_->steps_per_rev / cfg_->mm_per_rev);
    }
    float stepsToMm(int32_t steps) const {
        if (!cfg_ || cfg_->mm_per_rev <= 0.0f) return 0.0f;
        return (float)steps * cfg_->mm_per_rev / cfg_->steps_per_rev;
    }

    void setSoftLimits(int32_t min, int32_t max) {
        auto *c = const_cast<StepperMotorConfig *>(cfg_);
        c->soft_limit_min = min;
        c->soft_limit_max = max;
    }

private:
    const StepperMotorConfig *cfg_;

    volatile int32_t position_;
    volatile bool homed_;
    volatile bool moving_;
    volatile bool alarm_;
    bool enabled_;
    MotionDir cur_dir_;
    int32_t pending_steps_;

    AlarmCallback alarm_cb_;
    void *alarm_ctx_;

    struct k_sem done_sem_;

    uint64_t computeInterval(uint32_t rpm) const;
    int physicalDir(MotionDir logical) const;
    bool checkLimits(int32_t target_pos) const;

    static void stepperEventCb(const struct device *dev,
                               enum stepper_ctrl_event event, void *ud);
};

} // namespace component
