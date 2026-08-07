#pragma once

#include "stepper_motor.h"
#include "sensor.h"
#include "smema_upstream.h"
#include "smema_downstream.h"

namespace component {

class Conveyor {
public:
    struct Config {
        float belt_mm_per_rev;
        float width_mm_per_rev;
    };

    Conveyor() : belt_(nullptr), width_(nullptr),
                 laser1_(nullptr), laser2_(nullptr), laser3_(nullptr),
                 upstream_(nullptr), downstream_(nullptr), cfg_{0, 0} {}

    int init(StepperMotor *belt, StepperMotor *width,
             Sensor *laser1, Sensor *laser2, Sensor *laser3,
             SmemaUpstream *upstream, SmemaDownstream *downstream,
             const Config &cfg);

    // Belt motor
    void runBelt(float mm_per_sec, MotionDir dir);
    void stopBelt();
    StepperMotor *beltMotor() { return belt_; }

    // Width motor
    StepperMotor::Error setWidth(float mm, float speed_mm_s);
    float widthPosition_mm() const;
    StepperMotor *widthMotor() { return width_; }

    // Board position sensors
    bool boardPresented() const { return laser1_ && laser1_->triggered(); }
    bool boardNear() const { return laser2_ && laser2_->triggered(); }
    bool boardInPosition() const { return laser3_ && laser3_->triggered(); }
    Sensor *laser1() { return laser1_; }
    Sensor *laser2() { return laser2_; }
    Sensor *laser3() { return laser3_; }

    // SMEMA interfaces
    SmemaUpstream *upstream() { return upstream_; }
    SmemaDownstream *downstream() { return downstream_; }

    // Config
    Config &config() { return cfg_; }
    const Config &config() const { return cfg_; }
    float beltMmPerRev() const { return cfg_.belt_mm_per_rev; }
    float widthMmPerRev() const { return cfg_.width_mm_per_rev; }

    // Conversions
    uint32_t beltMmStoRpm(float mm_s) const;
    int32_t widthMmToSteps(float mm) const;
    float widthStepsToMm(int32_t steps) const;

private:
    StepperMotor *belt_;
    StepperMotor *width_;
    Sensor *laser1_;
    Sensor *laser2_;
    Sensor *laser3_;
    SmemaUpstream *upstream_;
    SmemaDownstream *downstream_;
    Config cfg_;
};

} // namespace component
