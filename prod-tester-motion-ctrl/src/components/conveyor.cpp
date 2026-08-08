#include "conveyor.h"

#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(conveyor, LOG_LEVEL_INF);

namespace component {

int Conveyor::init(StepperMotor *belt, StepperMotor *width,
                   Sensor *laser1, Sensor *laser2, Sensor *laser3,
                   SmemaUpstream *upstream, SmemaDownstream *downstream,
                   const Config &cfg)
{
    belt_ = belt;
    width_ = width;
    laser1_ = laser1;
    laser2_ = laser2;
    laser3_ = laser3;
    upstream_ = upstream;
    downstream_ = downstream;
    cfg_ = cfg;
    LOG_INF("Conveyor init: belt_mm_per_rev=%.2f width_mm_per_rev=%.2f",
            (double)cfg_.belt_mm_per_rev, (double)cfg_.width_mm_per_rev);
    return 0;
}

void Conveyor::runBelt(float mm_per_sec, MotionDir dir)
{
    uint32_t rpm = beltMmStoRpm(mm_per_sec);
    if (rpm == 0) rpm = 1;
    belt_->run(rpm, dir);
}

void Conveyor::stopBelt()
{
    belt_->stop();
}

StepperMotor::Error Conveyor::setWidth(float mm, float speed_mm_s)
{
    if (cfg_.width_mm_per_rev <= 0.0f) return StepperMotor::Error::NOT_READY;
    int32_t target = widthMmToSteps(mm);
    uint32_t rpm = (uint32_t)roundf(speed_mm_s * 60.0f / cfg_.width_mm_per_rev);
    if (rpm == 0) rpm = 1;
    return width_->goTo(target, rpm);
}

float Conveyor::widthPosition_mm() const
{
    return widthStepsToMm(width_->position());
}

uint32_t Conveyor::beltMmStoRpm(float mm_s) const
{
    if (cfg_.belt_mm_per_rev <= 0.0f) return 0;
    return (uint32_t)roundf(mm_s * 60.0f / cfg_.belt_mm_per_rev);
}

int32_t Conveyor::beltMmToSteps(float mm) const
{
    if (cfg_.belt_mm_per_rev <= 0.0f) return 0;
    return (int32_t)(mm * (float)belt_->config().steps_per_rev / cfg_.belt_mm_per_rev);
}

int32_t Conveyor::widthMmToSteps(float mm) const
{
    if (cfg_.width_mm_per_rev <= 0.0f) return 0;
    return (int32_t)(mm * (float)width_->config().steps_per_rev / cfg_.width_mm_per_rev);
}

float Conveyor::widthStepsToMm(int32_t steps) const
{
    if (cfg_.width_mm_per_rev <= 0.0f) return 0.0f;
    return (float)steps * cfg_.width_mm_per_rev / (float)width_->config().steps_per_rev;
}

} // namespace component
