#pragma once

#include "components/stepper_motor.h"
#include "components/cylinder.h"
#include "components/smema_port.h"
#include "components/sensor.h"
#include "engine/timer.h"
#include "config/recipe.h"

namespace engine {

enum class TestResult : uint8_t { NONE, PASS, FAIL };

struct Context {
    // Components
    component::StepperMotor *conveyor;      // axis1
    component::StepperMotor *width;         // axis2
    component::StepperMotor *table;         // axis3 (dual motor)

    component::Cylinder *stopper;           // solenoid 1
    component::Cylinder *rfid;              // solenoid 2
    component::Cylinder *locker;            // solenoid 3

    component::SmemaPort *smema;

    // Sequence-trigger sensors
    component::Sensor *laser1;              // panel presented (infeed)
    component::Sensor *laser2;              // panel near (mid)
    component::Sensor *laser3;              // panel in position (outfeed)

    // Cylinder confirm sensors
    component::Sensor *cyl1_a;
    component::Sensor *cyl1_b;
    component::Sensor *cyl2_a;
    component::Sensor *cyl2_b;
    component::Sensor *cyl3_a;
    component::Sensor *cyl3_b;

    // Table home sensor
    component::Sensor *table_home;

    // Active recipe
    config::Recipe *recipe;

    // Runtime state
    TestResult last_result;
    OperatingMode mode;
    uint32_t cycle_count;
    bool advance_requested;
    SoftTimer timer;

    // Trace: elapsed time tracking
    uint32_t wait_start_ms;
    const char *wait_desc;

    // Tester handshake
    volatile bool test_done;

    // SMEMA handoff: must see MR_IN go low before accepting high
    bool smema_seen_low;

    // Flags
    bool smema_enabled;
};

} // namespace engine
