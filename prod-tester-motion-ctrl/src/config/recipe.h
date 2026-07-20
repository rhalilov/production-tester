#pragma once

#include <stdint.h>

namespace config {

struct Recipe {
    static constexpr uint16_t CURRENT_VERSION = 1;

    uint16_t version;
    char name[32];

    // Motor speeds (7 presets, same as old mset)
    struct MotorPreset {
        uint32_t rpm;
        uint32_t steps;     // 0 = continuous (rpm-only)
    };
    MotorPreset motor_presets[7];

    // Cylinder timeout
    uint32_t cylinder_timeout_ms;

    // Conveyor
    uint32_t laser_debounce_ms;

    uint16_t crc;
};

struct FactoryConfig {
    static constexpr uint16_t CURRENT_VERSION = 1;

    uint16_t version;

    // Per-axis direction invert
    bool axis_invert[3];

    // Per-sensor active-high polarity
    bool sensor_active_high[11];

    // Sensor-to-role assignment (11 sensors -> roles)
    uint8_t sensor_roles[11];

    // Soft limits (axis 2 and 3)
    int32_t axis2_limit_min;
    int32_t axis2_limit_max;
    int32_t axis3_limit_min;
    int32_t axis3_limit_max;

    uint16_t crc;
};

} // namespace config
