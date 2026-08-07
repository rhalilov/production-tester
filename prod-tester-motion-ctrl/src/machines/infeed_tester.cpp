// Machine: Infeed PCB Tester (Shelly)
// Hardware instances + SFC sequence definition

#include "engine/engine.h"
#include "engine/context.h"
#include "engine/trace.h"
#include "engine/step.h"
#include "components/stepper_motor.h"
#include "components/cylinder.h"
#include "components/conveyor.h"
#include "components/head.h"
#include "components/smema_upstream.h"
#include "components/smema_downstream.h"
#include "components/sensor.h"
#include "config/recipe.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(infeed, LOG_LEVEL_INF);

using namespace component;
using namespace engine;

// --- Hardware pin specs from device tree ---

// ENA pins
static const gpio_dt_spec ena1_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(ena1), gpios);
static const gpio_dt_spec ena2_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(ena2), gpios);
static const gpio_dt_spec ena3_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(ena3), gpios);

// Solenoid pins
static const gpio_dt_spec sol1a_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(sol1_a), gpios);
static const gpio_dt_spec sol1b_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(sol1_b), gpios);
static const gpio_dt_spec sol2a_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(sol2_a), gpios);
static const gpio_dt_spec sol2b_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(sol2_b), gpios);
static const gpio_dt_spec sol3a_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(sol3_a), gpios);
static const gpio_dt_spec sol3b_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(sol3_b), gpios);

// ALM pins
#define USER_NODE DT_PATH(zephyr_user)
static const gpio_dt_spec alm1_spec = GPIO_DT_SPEC_GET(USER_NODE, alm1_gpios);
static const gpio_dt_spec alm2_spec = GPIO_DT_SPEC_GET(USER_NODE, alm2_gpios);
static const gpio_dt_spec alm3_spec = GPIO_DT_SPEC_GET(USER_NODE, alm3_gpios);
static const gpio_dt_spec alm4_spec = GPIO_DT_SPEC_GET(USER_NODE, alm4_gpios);

// PEND pins (only head motors 3+4)
static const gpio_dt_spec pend3_spec = GPIO_DT_SPEC_GET(USER_NODE, pend3_gpios);
static const gpio_dt_spec pend4_spec = GPIO_DT_SPEC_GET(USER_NODE, pend4_gpios);

// Sensor pins
static const gpio_dt_spec laser1_spec = GPIO_DT_SPEC_GET(USER_NODE, laser1_gpios);
static const gpio_dt_spec laser2_spec = GPIO_DT_SPEC_GET(USER_NODE, laser2_gpios);
static const gpio_dt_spec laser3_spec = GPIO_DT_SPEC_GET(USER_NODE, laser3_gpios);
static const gpio_dt_spec ind4_spec   = GPIO_DT_SPEC_GET(USER_NODE, inductive4_gpios);
static const gpio_dt_spec ind5_spec   = GPIO_DT_SPEC_GET(USER_NODE, inductive5_gpios);
static const gpio_dt_spec ind6_spec   = GPIO_DT_SPEC_GET(USER_NODE, inductive6_gpios);
static const gpio_dt_spec ind7_spec   = GPIO_DT_SPEC_GET(USER_NODE, inductive7_gpios);
static const gpio_dt_spec ind8_spec   = GPIO_DT_SPEC_GET(USER_NODE, inductive8_gpios);
static const gpio_dt_spec ind9_spec   = GPIO_DT_SPEC_GET(USER_NODE, inductive9_gpios);
static const gpio_dt_spec photo10_spec = GPIO_DT_SPEC_GET(USER_NODE, photo10_gpios);

// SMEMA pins
static const gpio_dt_spec smema_up_ba_spec = GPIO_DT_SPEC_GET(USER_NODE, smema_up_board_available_gpios);
static const gpio_dt_spec smema_up_mr_spec = GPIO_DT_SPEC_GET(USER_NODE, smema_up_machine_ready_gpios);
static const gpio_dt_spec smema_dn_ba_spec = GPIO_DT_SPEC_GET(USER_NODE, smema_down_board_available_gpios);
static const gpio_dt_spec smema_dn_mr_spec = GPIO_DT_SPEC_GET(USER_NODE, smema_down_machine_ready_gpios);
static const gpio_dt_spec smema_dn_ba_fail_spec = GPIO_DT_SPEC_GET(USER_NODE, smema_down_board_available_fail_gpios);

// --- Component configurations ---

static const gpio_dt_spec alm_axis1[] = { alm1_spec };
static const gpio_dt_spec alm_axis2[] = { alm2_spec };
static const gpio_dt_spec alm_axis3[] = { alm3_spec, alm4_spec };
static const gpio_dt_spec pend_axis3[] = { pend3_spec, pend4_spec };

static StepperMotorConfig conveyor_cfg = {
    .stepper_dev = DEVICE_DT_GET(DT_NODELABEL(axis1)),
    .ena_pin = &ena1_spec,
    .alm_pins = alm_axis1,
    .alm_count = 1,
    .alm_policy = AlmPolicy::ANY_TRIGGERS_FAULT,
    .pend_pins = nullptr,
    .pend_count = 0,
    .pend_policy = PendPolicy::ALL_REQUIRED,
    .has_home = false,
    .home_sensor = nullptr,
    .home_dir = MotionDir::POS,
    .auto_home = false,
    .home_rpm = 0,
    .has_limits = false,
    .soft_limit_min = 0,
    .soft_limit_max = 0,
    .safe_position = 0,
    .persist_position = false,
    .invert_dir = false,    // conveyor direction
    .accel = { .start_rpm = 5, .accel_rpm_s = 100 },
    .steps_per_rev = CONFIG_MOTOR_STEPS_PER_REV,
};

static StepperMotorConfig width_cfg = {
    .stepper_dev = DEVICE_DT_GET(DT_NODELABEL(axis2)),
    .ena_pin = &ena2_spec,
    .alm_pins = alm_axis2,
    .alm_count = 1,
    .alm_policy = AlmPolicy::ANY_TRIGGERS_FAULT,
    .pend_pins = nullptr,
    .pend_count = 0,
    .pend_policy = PendPolicy::ALL_REQUIRED,
    .has_home = true,
    .home_sensor = nullptr,   // width home TBD
    .home_dir = MotionDir::NEG,
    .auto_home = false,
    .home_rpm = 10,
    .has_limits = true,
    .soft_limit_min = 0,
    .soft_limit_max = 50000,   // TBD: calibrate
    .safe_position = 0,
    .persist_position = true,
    .invert_dir = false,
    .accel = { .start_rpm = 5, .accel_rpm_s = 80 },
    .steps_per_rev = CONFIG_MOTOR_STEPS_PER_REV,
};

static StepperMotorConfig head_motor_cfg = {
    .stepper_dev = DEVICE_DT_GET(DT_NODELABEL(axis3)),
    .ena_pin = &ena3_spec,
    .alm_pins = alm_axis3,
    .alm_count = 2,
    .alm_policy = AlmPolicy::ANY_TRIGGERS_FAULT,
    .pend_pins = pend_axis3,
    .pend_count = 2,
    .pend_policy = PendPolicy::ALL_REQUIRED,
    .has_home = true,
    .home_sensor = &photo10_spec,
    .home_dir = MotionDir::POS,
    .auto_home = true,
    .home_rpm = 10,
    .has_limits = true,
    .soft_limit_min = -15000,  // max head down (steps from home)
    .soft_limit_max = 0,       // home = top
    .safe_position = 0,        // safe = home (top)
    .persist_position = true,
    .invert_dir = false,
    .accel = { .start_rpm = 5, .accel_rpm_s = 120 },
    .steps_per_rev = CONFIG_MOTOR_STEPS_PER_REV,
};

static CylinderConfig stopper_cfg = {
    .name = "stopper",
    .coil_a = &sol1a_spec,
    .coil_b = &sol1b_spec,
    .sensor_a = &ind5_spec,
    .sensor_b = &ind4_spec,
    .timeout_ms = CONFIG_CYLINDER_CONFIRM_TIMEOUT_MS,
};

static CylinderConfig rfid_cfg = {
    .name = "rfid",
    .coil_a = &sol2a_spec,
    .coil_b = &sol2b_spec,
    .sensor_a = &ind6_spec,
    .sensor_b = &ind7_spec,
    .timeout_ms = CONFIG_CYLINDER_CONFIRM_TIMEOUT_MS,
};

static CylinderConfig locker_cfg = {
    .name = "locker",
    .coil_a = &sol3a_spec,
    .coil_b = &sol3b_spec,
    .sensor_a = &ind9_spec,
    .sensor_b = &ind8_spec,
    .timeout_ms = CONFIG_CYLINDER_CONFIRM_TIMEOUT_MS,
};

static SmemaUpstreamConfig smema_up_cfg = {
    .ba_in = &smema_up_ba_spec,
    .mr_out = &smema_up_mr_spec,
};

static SmemaDownstreamConfig smema_dn_cfg = {
    .mr_in = &smema_dn_mr_spec,
    .ba_out = &smema_dn_ba_spec,
    .ba_fail_out = &smema_dn_ba_fail_spec,
};

// Sensor configs
static SensorConfig laser1_cfg = { &laser1_spec, "laser1", 50, true };
static SensorConfig laser2_cfg = { &laser2_spec, "laser2", 50, true };
static SensorConfig laser3_cfg = { &laser3_spec, "laser3", 50, true };
static SensorConfig head_home_cfg = { &photo10_spec, "head_home", 0, true };
static SensorConfig cyl1a_cfg = { &ind6_spec, "ind6", 0, true };
static SensorConfig cyl1b_cfg = { &ind7_spec, "ind7", 0, true };
static SensorConfig cyl2a_cfg = { &ind8_spec, "ind8", 0, true };
static SensorConfig cyl2b_cfg = { &ind9_spec, "ind9", 0, true };
static SensorConfig cyl3a_cfg = { &ind4_spec, "ind4", 0, true };
static SensorConfig cyl3b_cfg = { &ind5_spec, "ind5", 0, true };

// --- Component instances ---

static StepperMotor motor_conveyor;
static StepperMotor motor_width;
static StepperMotor motor_head;

static Cylinder cyl_stopper;
static Cylinder cyl_rfid;
static Cylinder cyl_locker;

static SmemaUpstream smema_upstream;
static SmemaDownstream smema_downstream;

static Sensor sens_laser1;
static Sensor sens_laser2;
static Sensor sens_laser3;
static Sensor sens_head_home;
static Sensor sens_cyl1a, sens_cyl1b;
static Sensor sens_cyl2a, sens_cyl2b;
static Sensor sens_cyl3a, sens_cyl3b;

static Conveyor machine_conveyor;
static Head machine_head;

// --- Default recipe ---
static config::Recipe default_recipe = {
    .version = config::Recipe::CURRENT_VERSION,
    .name = "default",
    .motor_presets = {
        { 10, 0 },                                       // [0] home/up rpm (unused, use motor config)
        { 40, 0 },                                       // [1] convey rpm
        { 10, CONFIG_MOTOR_STEPS_PER_REV / 2 },          // [2] creep rpm + steps
        { 200, 0 },                                      // [3] head fast rpm
        { 40, 0 },                                       // [4] head slow rpm
        { 40, 0 },                                       // [5] convey out rpm
        { 20, 0 },                                       // [6] eject rpm
    },
    .head_pos = {
        .guides_clear = -10.0f,     // mm — guides out of board
        .pins_touch   = -25.0f,     // mm — probes just touching
        .pins_contact = -30.0f,     // mm — full contact (test)
    },
    .cylinder_timeout_ms = CONFIG_CYLINDER_CONFIRM_TIMEOUT_MS,
    .laser_debounce_ms = 50,
    .crc = 0,
};

// --- SFC Sequence ---

static void interlock_check(Context &ctx, FaultManager &faults)
{
    // Poll sensors every scan
    ctx.conveyor->laser1()->poll();
    ctx.conveyor->laser2()->poll();
    ctx.conveyor->laser3()->poll();
    ctx.head->homeSensor()->poll();
    ctx.cyl1_a->poll();
    ctx.cyl1_b->poll();
    ctx.cyl2_a->poll();
    ctx.cyl2_b->poll();
    ctx.cyl3_a->poll();
    ctx.cyl3_b->poll();

    // Check ALM signals
    if (motor_conveyor.inAlarm() || motor_width.inAlarm() || motor_head.inAlarm()) {
        faults.raise("Motor ALM asserted");
    }
}

// Sequence steps (simplified — full 32-step sequence to be expanded)
static StepDef infeed_steps[] = {
    // Step 0: Idle — wait for downstream signal to go LOW (board request)
    { "idle",
      [](Context &ctx) {
          TRACE_WAIT("downstream MR_IN low");
          ctx.wait_desc = "downstream MR_IN low";
      },
      nullptr,
      nullptr,
      [](Context &ctx) -> bool {
          return !ctx.conveyor->upstream()->boardAvailable();
      },
      1, nullptr, 0
    },

    // Step 1: Request board — signal ready + run conveyor, wait for laser1
    { "request",
      [](Context &ctx) {
          TRACE_ACT("smema_mr_out=HIGH");
          ctx.conveyor->upstream()->setMachineReady(true);
          TRACE_ACT("conveyor RUN +%urpm", ctx.recipe->motor_presets[1].rpm);
          ctx.conveyor->beltMotor()->run(ctx.recipe->motor_presets[1].rpm, MotionDir::POS);
          TRACE_WAIT("laser1 triggered");
          ctx.wait_desc = "laser1 triggered";
      },
      nullptr,
      nullptr,
      [](Context &ctx) -> bool {
          return ctx.conveyor->boardPresented();
      },
      2, nullptr, 0
    },

    // Step 2: Board entering — conveyor keeps running, wait until fully past laser1
    { "wait_panel",
      [](Context &ctx) {
          TRACE_WAIT("laser1 released");
          ctx.wait_desc = "laser1 released";
      },
      nullptr,
      [](Context &ctx) {
          TRACE_ACT("smema_mr_out=LOW");
          ctx.conveyor->upstream()->setMachineReady(false);
          TRACE_ACT("conveyor STOP");
          ctx.conveyor->stopBelt();
      },
      [](Context &ctx) -> bool { return !ctx.conveyor->boardPresented(); },
      3, nullptr, 0
    },

    // Step 3: Convey to near position (laser2)
    { "convey_in",
      [](Context &ctx) {
          TRACE_ACT("conveyor RUN +%urpm", ctx.recipe->motor_presets[1].rpm);
          ctx.conveyor->beltMotor()->run(ctx.recipe->motor_presets[1].rpm, MotionDir::POS);
          TRACE_WAIT("laser2 triggered");
          ctx.wait_desc = "laser2 triggered";
      },
      nullptr,
      [](Context &ctx) {
          TRACE_ACT("conveyor STOP");
          ctx.conveyor->stopBelt();
      },
      [](Context &ctx) -> bool { return ctx.conveyor->boardNear(); },
      4, nullptr, 0
    },

    // Step 4: Arm stopper
    { "arm_stopper",
      [](Context &ctx) {
          TRACE_ACT("stopper -> POS_A");
          ctx.stopper->goTo(CylPosition::POS_A);
          TRACE_WAIT("stopper at POS_A");
          ctx.wait_desc = "stopper at POS_A";
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.stopper->currentPos() == CylPosition::POS_A; },
      5, nullptr, 0
    },

    // Step 5: Creep into stopper
    { "creep",
      [](Context &ctx) {
          TRACE_ACT("conveyor GO %d steps @%urpm",
                    ctx.recipe->motor_presets[2].steps, ctx.recipe->motor_presets[2].rpm);
          ctx.conveyor->beltMotor()->go(ctx.recipe->motor_presets[2].steps, ctx.recipe->motor_presets[2].rpm);
          TRACE_WAIT("conveyor move done");
          ctx.wait_desc = "conveyor move done";
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return !ctx.conveyor->beltMotor()->isMoving(); },
      6, nullptr, 0
    },

    // Step 6: Head down to pins_touch
    { "pins_touch",
      [](Context &ctx) {
          float target_mm = ctx.recipe->head_pos.pins_touch;
          int32_t target = ctx.head->mmToSteps(target_mm);
          TRACE_ACT("head GOTO %.1fmm (%d steps) @%urpm",
                    (double)target_mm, target,
                    ctx.recipe->motor_presets[3].rpm);
          ctx.head->motor()->goTo(target, ctx.recipe->motor_presets[3].rpm);
          TRACE_WAIT("head move done");
          ctx.wait_desc = "head move done";
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return !ctx.head->isMoving(); },
      7, nullptr, 0
    },

    // Step 7: Head down to pins_contact
    { "pins_contact",
      [](Context &ctx) {
          float target_mm = ctx.recipe->head_pos.pins_contact;
          int32_t target = ctx.head->mmToSteps(target_mm);
          TRACE_ACT("head GOTO %.1fmm (%d steps) @%urpm",
                    (double)target_mm, target,
                    ctx.recipe->motor_presets[4].rpm);
          ctx.head->motor()->goTo(target, ctx.recipe->motor_presets[4].rpm);
          TRACE_WAIT("head move done");
          ctx.wait_desc = "head move done";
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return !ctx.head->isMoving(); },
      8, nullptr, 0
    },

    // Step 8: Arm RFID
    { "arm_rfid",
      [](Context &ctx) {
          TRACE_ACT("rfid -> POS_A");
          ctx.rfid->goTo(CylPosition::POS_A);
          TRACE_WAIT("rfid at POS_A");
          ctx.wait_desc = "rfid at POS_A";
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.rfid->currentPos() == CylPosition::POS_A; },
      9, nullptr, 0
    },

    // Step 9: Arm panel locker
    { "arm_locker",
      [](Context &ctx) {
          TRACE_ACT("locker -> POS_B");
          ctx.locker->goTo(CylPosition::POS_B);
          TRACE_WAIT("locker at POS_B");
          ctx.wait_desc = "locker at POS_B";
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.locker->currentPos() == CylPosition::POS_B; },
      10, nullptr, 0
    },

    // Step 10: Test/Program — wait for tester command
    { "testing",
      [](Context &ctx) {
          ctx.test_done = false;
          ctx.last_result = TestResult::NONE;
          TRACE_WAIT("test done");
          ctx.wait_desc = "test done";
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool {
          return ctx.test_done;
      },
      11, nullptr, 0
    },

    // Step 11: Table up to home
    { "head_up",
      [](Context &ctx) {
          uint32_t rpm = ctx.head->motor()->config().home_rpm;
          TRACE_ACT("head RUN +%urpm (up to home)", rpm);
          ctx.head->motor()->run(rpm, MotionDir::POS);
          TRACE_WAIT("head_home triggered");
          ctx.wait_desc = "head_home triggered";
      },
      nullptr,
      [](Context &ctx) {
          TRACE_ACT("head STOP");
          ctx.head->motor()->stop();
      },
      [](Context &ctx) -> bool { return ctx.head->atHome(); },
      12, nullptr, 0
    },

    // Step 12: Release RFID
    { "release_rfid",
      [](Context &ctx) {
          TRACE_ACT("rfid -> POS_B");
          ctx.rfid->goTo(CylPosition::POS_B);
          TRACE_WAIT("rfid at POS_B");
          ctx.wait_desc = "rfid at POS_B";
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.rfid->currentPos() == CylPosition::POS_B; },
      13, nullptr, 0
    },

    // Step 13: Release locker
    { "release_locker",
      [](Context &ctx) {
          TRACE_ACT("locker -> POS_A");
          ctx.locker->goTo(CylPosition::POS_A);
          TRACE_WAIT("locker at POS_A");
          ctx.wait_desc = "locker at POS_A";
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.locker->currentPos() == CylPosition::POS_A; },
      14, nullptr, 0
    },

    // Step 14: Disarm stopper
    { "disarm_stopper",
      [](Context &ctx) {
          TRACE_ACT("stopper -> POS_B");
          ctx.stopper->goTo(CylPosition::POS_B);
          TRACE_WAIT("stopper at POS_B");
          ctx.wait_desc = "stopper at POS_B";
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.stopper->currentPos() == CylPosition::POS_B; },
      15, nullptr, 0
    },

    // Step 15: Convey out
    { "convey_out",
      [](Context &ctx) {
          TRACE_ACT("conveyor RUN +%urpm", ctx.recipe->motor_presets[5].rpm);
          ctx.conveyor->beltMotor()->run(ctx.recipe->motor_presets[5].rpm, MotionDir::POS);
          TRACE_WAIT("laser3 triggered");
          ctx.wait_desc = "laser3 triggered";
      },
      nullptr,
      [](Context &ctx) {
          TRACE_ACT("conveyor STOP");
          ctx.conveyor->stopBelt();
      },
      [](Context &ctx) -> bool { return ctx.conveyor->boardInPosition(); },
      16, nullptr, 0
    },

    // Step 16: SMEMA handoff downstream — wait for next machine to request board
    { "smema_out",
      [](Context &ctx) {
          TRACE_ACT("conveyor STOPPED at laser3, board ready");
          if (ctx.last_result == TestResult::FAIL) {
              TRACE_ACT("smema_ba_fail_out=HIGH (NG)");
              ctx.conveyor->downstream()->setBoardAvailableFail(true);
          } else {
              TRACE_ACT("smema_ba_out=HIGH");
              ctx.conveyor->downstream()->setBoardAvailable(true);
          }
          TRACE_WAIT("Down MR (next machine request)");
          ctx.wait_desc = "Down MR (next machine request)";
      },
      nullptr,
      [](Context &ctx) {
          TRACE_ACT("smema outputs LOW");
          ctx.conveyor->downstream()->allOff();
      },
      [](Context &ctx) -> bool {
          return !ctx.conveyor->downstream()->machineReady();
      },
      17, nullptr, 0
    },

    // Step 17: Eject — run until laser3 clears
    { "eject",
      [](Context &ctx) {
          TRACE_ACT("conveyor RUN +%urpm (eject)", ctx.recipe->motor_presets[6].rpm);
          ctx.conveyor->beltMotor()->run(ctx.recipe->motor_presets[6].rpm, MotionDir::POS);
          TRACE_WAIT("laser3 released");
          ctx.wait_desc = "laser3 released";
      },
      nullptr,
      [](Context &ctx) {
          TRACE_ACT("conveyor STOP");
          ctx.conveyor->stopBelt();
      },
      [](Context &ctx) -> bool { return !ctx.conveyor->boardInPosition(); },
      -1, nullptr, 0
    },
};

static constexpr int INFEED_STEP_COUNT = sizeof(infeed_steps) / sizeof(infeed_steps[0]);

// --- Context + Engine config ---

Context machine_context = {};

EngineConfig machine_engine_config = {
    .steps = infeed_steps,
    .step_count = INFEED_STEP_COUNT,
    .interlock_check = interlock_check,
    .scan_period_ms = CONFIG_SCAN_PERIOD_MS,
};

// --- Machine init ---

int machine_init(void)
{
    int err;

    // Motors
    err = motor_conveyor.init(conveyor_cfg);
    if (err) { LOG_ERR("conveyor init: %d", err); return err; }

    err = motor_width.init(width_cfg);
    if (err) { LOG_ERR("width init: %d", err); return err; }

    err = motor_head.init(head_motor_cfg);
    if (err) { LOG_ERR("head init: %d", err); return err; }

    // Cylinders
    err = cyl_stopper.init(stopper_cfg);
    if (err) { LOG_ERR("stopper init: %d", err); return err; }

    err = cyl_rfid.init(rfid_cfg);
    if (err) { LOG_ERR("rfid init: %d", err); return err; }

    err = cyl_locker.init(locker_cfg);
    if (err) { LOG_ERR("locker init: %d", err); return err; }

    // SMEMA
    err = smema_upstream.init(smema_up_cfg);
    if (err) { LOG_ERR("smema upstream init: %d", err); return err; }

    err = smema_downstream.init(smema_dn_cfg);
    if (err) { LOG_ERR("smema downstream init: %d", err); return err; }

    // Sensors
    sens_laser1.init(laser1_cfg);
    sens_laser2.init(laser2_cfg);
    sens_laser3.init(laser3_cfg);
    sens_head_home.init(head_home_cfg);
    sens_cyl1a.init(cyl1a_cfg);
    sens_cyl1b.init(cyl1b_cfg);
    sens_cyl2a.init(cyl2a_cfg);
    sens_cyl2b.init(cyl2b_cfg);
    sens_cyl3a.init(cyl3a_cfg);
    sens_cyl3b.init(cyl3b_cfg);

    // High-level assemblies
    Conveyor::Config conv_cfg = { .belt_mm_per_rev = 0, .width_mm_per_rev = 4.0f };
    machine_conveyor.init(&motor_conveyor, &motor_width,
                          &sens_laser1, &sens_laser2, &sens_laser3,
                          &smema_upstream, &smema_downstream,
                          conv_cfg);

    Head::Config head_init_cfg = { .mm_per_rev = 5.0f };
    machine_head.init(&motor_head, &sens_head_home, head_init_cfg);

    // Wire up context
    machine_context.conveyor = &machine_conveyor;
    machine_context.head = &machine_head;
    machine_context.stopper = &cyl_stopper;
    machine_context.rfid = &cyl_rfid;
    machine_context.locker = &cyl_locker;
    machine_context.cyl1_a = &sens_cyl1a;
    machine_context.cyl1_b = &sens_cyl1b;
    machine_context.cyl2_a = &sens_cyl2a;
    machine_context.cyl2_b = &sens_cyl2b;
    machine_context.cyl3_a = &sens_cyl3a;
    machine_context.cyl3_b = &sens_cyl3b;
    machine_context.recipe = &default_recipe;
    machine_context.last_result = TestResult::NONE;
    machine_context.mode = OperatingMode::AUTO;
    machine_context.cycle_count = 0;
    machine_context.advance_requested = false;
    machine_context.smema_enabled = true;

    // Enable motors
    motor_conveyor.setEnabled(true);
    motor_width.setEnabled(true);
    motor_head.setEnabled(true);

    // Auto-home head at boot (moved to main.cpp after settings load)
    // if (head_cfg.auto_home) { ... }

    LOG_INF("Infeed tester machine init complete (%d steps)", INFEED_STEP_COUNT);
    return 0;
}
