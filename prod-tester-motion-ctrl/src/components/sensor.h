#pragma once

#include <stdint.h>
#include <zephyr/drivers/gpio.h>

namespace component {

struct SensorConfig {
    const gpio_dt_spec *pin;
    const char *name;
    uint32_t debounce_ms;   // 0 = no debounce
    bool active_high;       // runtime-configurable polarity
};

class Sensor {
public:
    Sensor() : cfg_(nullptr), state_(false), raw_(false),
               stable_since_(0), last_change_(0) {}

    int init(const SensorConfig &cfg);

    // Call periodically (from scan loop) to update debounced state
    void poll();

    bool triggered() const { return state_; }
    bool raw() const { return raw_; }
    const char *name() const { return cfg_ ? cfg_->name : "?"; }

    void setActiveHigh(bool ah);
    bool activeHigh() const { return cfg_ ? cfg_->active_high : true; }

    const SensorConfig &config() const { return *cfg_; }

private:
    const SensorConfig *cfg_;
    bool state_;
    bool raw_;
    uint32_t stable_since_;
    uint32_t last_change_;

    bool readPhysical() const;
};

} // namespace component
