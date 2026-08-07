#pragma once

#include "components/conveyor.h"
#include "components/head.h"
#include "components/cylinder.h"
#include "engine/timer.h"
#include "config/recipe.h"

namespace engine {

enum class TestResult : uint8_t { NONE, PASS, FAIL };

struct Context {
    // High-level assemblies
    component::Conveyor *conveyor;
    component::Head *head;

    // Cylinders
    component::Cylinder *stopper;           // solenoid 1
    component::Cylinder *rfid;              // solenoid 2
    component::Cylinder *locker;            // solenoid 3

    // Cylinder confirm sensors
    component::Sensor *cyl1_a;
    component::Sensor *cyl1_b;
    component::Sensor *cyl2_a;
    component::Sensor *cyl2_b;
    component::Sensor *cyl3_a;
    component::Sensor *cyl3_b;

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
