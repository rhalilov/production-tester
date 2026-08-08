#pragma once

#include "stepper_motor.h"
#include "sensor.h"

namespace component {

class Head {
public:
    struct Config {
        float mm_per_rev;
        float fast_speed;
        float slow_speed;
    };

    Head() : motor_(nullptr), home_sensor_(nullptr), cfg_{0} {}

    int init(StepperMotor *motor, Sensor *home_sensor, const Config &cfg);

    StepperMotor::Error goTo(float mm, float speed_mm_s);
    int home();

    float position_mm() const;
    bool isMoving() const { return motor_->isMoving(); }
    bool isHomed() const { return motor_->isHomed(); }
    bool atHome() const { return home_sensor_->triggered(); }

    StepperMotor *motor() { return motor_; }
    Sensor *homeSensor() { return home_sensor_; }

    Config &config() { return cfg_; }
    const Config &config() const { return cfg_; }
    float mmPerRev() const { return cfg_.mm_per_rev; }

    int32_t mmToSteps(float mm) const;
    float stepsToMm(int32_t steps) const;
    uint32_t mmStoRpm(float mm_s) const;

private:
    StepperMotor *motor_;
    Sensor *home_sensor_;
    Config cfg_;
};

} // namespace component
