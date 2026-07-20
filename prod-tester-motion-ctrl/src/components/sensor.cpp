#include "sensor.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor, LOG_LEVEL_DBG);

namespace component {

int Sensor::init(const SensorConfig &cfg)
{
    cfg_ = &cfg;
    state_ = false;
    raw_ = false;
    stable_since_ = 0;
    last_change_ = 0;

    if (!cfg_->pin || !gpio_is_ready_dt(cfg_->pin)) {
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(cfg_->pin, GPIO_INPUT);
    if (err) {
        return err;
    }

    raw_ = readPhysical();
    state_ = raw_;
    return 0;
}

bool Sensor::readPhysical() const
{
    int val = gpio_pin_get_dt(cfg_->pin);
    if (cfg_->active_high) {
        return val != 0;
    }
    return val == 0;
}

void Sensor::poll()
{
    if (!cfg_) return;

    bool current = readPhysical();
    raw_ = current;

    if (cfg_->debounce_ms == 0) {
        state_ = current;
        return;
    }

    uint32_t now = k_uptime_get_32();

    if (current != state_) {
        if (stable_since_ == 0) {
            stable_since_ = now;
        } else if ((now - stable_since_) >= cfg_->debounce_ms) {
            state_ = current;
            stable_since_ = 0;
            last_change_ = now;
        }
    } else {
        stable_since_ = 0;
    }
}

void Sensor::setActiveHigh(bool ah)
{
    if (cfg_) {
        const_cast<SensorConfig *>(cfg_)->active_high = ah;
    }
}

} // namespace component
