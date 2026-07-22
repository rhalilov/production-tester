#pragma once

#include <stdint.h>
#include <zephyr/drivers/gpio.h>

namespace component {

enum class CylPosition : uint8_t { POS_A, POS_B, UNKNOWN };

struct CylinderConfig {
    const gpio_dt_spec *coil_a;
    const gpio_dt_spec *coil_b;
    const gpio_dt_spec *sensor_a;
    const gpio_dt_spec *sensor_b;
    uint32_t timeout_ms;        // confirm timeout (0 = infinite)
};

class Cylinder {
public:
    enum class Error {
        OK = 0,
        TIMEOUT = -1,
        INTERRUPTED = -2,
        NOT_READY = -3,
    };

    Cylinder() : cfg_(nullptr), interrupted_(false) {}

    int init(const CylinderConfig &cfg);

    // Blocking: energize coil, wait for confirm sensor, then coils off.
    Error goTo(CylPosition pos);

    CylPosition currentPos() const;

    // Coils off (safe state for bistable valve = locked in last position)
    void off();

    void setTimeout(uint32_t ms);
    void interrupt() { interrupted_ = true; }
    void clearInterrupt() { interrupted_ = false; }

    // Direct access to modify sensor mapping
    void setSensorA(const gpio_dt_spec *pin) {
        if (cfg_) const_cast<CylinderConfig *>(cfg_)->sensor_a = pin;
    }
    void setSensorB(const gpio_dt_spec *pin) {
        if (cfg_) const_cast<CylinderConfig *>(cfg_)->sensor_b = pin;
    }

    const CylinderConfig &config() const { return *cfg_; }

private:
    const CylinderConfig *cfg_;
    volatile bool interrupted_;

    bool sensorA() const;
    bool sensorB() const;
};

} // namespace component
