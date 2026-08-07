#include "head.h"

#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(head, LOG_LEVEL_INF);

namespace component {

int Head::init(StepperMotor *motor, Sensor *home_sensor, const Config &cfg)
{
    motor_ = motor;
    home_sensor_ = home_sensor;
    cfg_ = cfg;
    LOG_INF("Head init: mm_per_rev=%.2f", (double)cfg_.mm_per_rev);
    return 0;
}

StepperMotor::Error Head::goTo(float mm, float speed_mm_s)
{
    if (cfg_.mm_per_rev <= 0.0f) return StepperMotor::Error::NOT_READY;
    int32_t target = mmToSteps(mm);
    uint32_t rpm = mmStoRpm(speed_mm_s);
    if (rpm == 0) rpm = 1;
    return motor_->goTo(target, rpm);
}

int Head::home()
{
    return motor_->home();
}

float Head::position_mm() const
{
    return stepsToMm(motor_->position());
}

int32_t Head::mmToSteps(float mm) const
{
    if (cfg_.mm_per_rev <= 0.0f) return 0;
    return (int32_t)(mm * (float)motor_->config().steps_per_rev / cfg_.mm_per_rev);
}

float Head::stepsToMm(int32_t steps) const
{
    if (cfg_.mm_per_rev <= 0.0f) return 0.0f;
    return (float)steps * cfg_.mm_per_rev / (float)motor_->config().steps_per_rev;
}

uint32_t Head::mmStoRpm(float mm_s) const
{
    if (cfg_.mm_per_rev <= 0.0f) return 0;
    return (uint32_t)roundf(mm_s * 60.0f / cfg_.mm_per_rev);
}

} // namespace component
