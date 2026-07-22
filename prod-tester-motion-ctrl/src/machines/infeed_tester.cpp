// Machine: Infeed PCB Tester (Shelly)
// Hardware instances + SFC sequence definition

#include "engine/engine.h"
#include "engine/context.h"
#include "engine/step.h"
#include "components/stepper_motor.h"
#include "components/cylinder.h"
#include "components/smema_port.h"
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
    .mm_per_rev = 0,    // conveyor: no linear axis
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
    .mm_per_rev = 4.0f,    // width: 4mm per revolution
};

static StepperMotorConfig table_cfg = {
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
    .soft_limit_min = -15000,  // max table down (steps from home)
    .soft_limit_max = 0,       // home = top
    .safe_position = 0,        // safe = home (top)
    .persist_position = true,
    .invert_dir = false,
    .accel = { .start_rpm = 5, .accel_rpm_s = 120 },
    .steps_per_rev = CONFIG_MOTOR_STEPS_PER_REV,
    .mm_per_rev = 5.0f,    // table: 5mm per revolution
};

static CylinderConfig stopper_cfg = {
    .coil_a = &sol1a_spec,
    .coil_b = &sol1b_spec,
    .sensor_a = &ind6_spec,
    .sensor_b = &ind7_spec,
    .timeout_ms = CONFIG_CYLINDER_CONFIRM_TIMEOUT_MS,
};

static CylinderConfig rfid_cfg = {
    .coil_a = &sol2a_spec,
    .coil_b = &sol2b_spec,
    .sensor_a = &ind8_spec,
    .sensor_b = &ind9_spec,
    .timeout_ms = CONFIG_CYLINDER_CONFIRM_TIMEOUT_MS,
};

static CylinderConfig locker_cfg = {
    .coil_a = &sol3a_spec,
    .coil_b = &sol3b_spec,
    .sensor_a = &ind4_spec,
    .sensor_b = &ind5_spec,
    .timeout_ms = CONFIG_CYLINDER_CONFIRM_TIMEOUT_MS,
};

static SmemaPortConfig smema_cfg = {
    .board_available_in = &smema_up_ba_spec,
    .machine_ready_out = &smema_up_mr_spec,
    .board_available_out = &smema_dn_ba_spec,
    .machine_ready_in = &smema_dn_mr_spec,
    .board_available_fail_out = &smema_dn_ba_fail_spec,
};

// Sensor configs
static SensorConfig laser1_cfg = { &laser1_spec, "laser1", 50, false };
static SensorConfig laser2_cfg = { &laser2_spec, "laser2", 50, false };
static SensorConfig laser3_cfg = { &laser3_spec, "laser3", 50, false };
static SensorConfig table_home_cfg = { &photo10_spec, "table_home", 0, true };
static SensorConfig cyl1a_cfg = { &ind6_spec, "cyl1_a", 0, true };
static SensorConfig cyl1b_cfg = { &ind7_spec, "cyl1_b", 0, true };
static SensorConfig cyl2a_cfg = { &ind8_spec, "cyl2_a", 0, true };
static SensorConfig cyl2b_cfg = { &ind9_spec, "cyl2_b", 0, true };
static SensorConfig cyl3a_cfg = { &ind4_spec, "cyl3_a", 0, true };
static SensorConfig cyl3b_cfg = { &ind5_spec, "cyl3_b", 0, true };

// --- Component instances ---

static StepperMotor motor_conveyor;
static StepperMotor motor_width;
static StepperMotor motor_table;

static Cylinder cyl_stopper;
static Cylinder cyl_rfid;
static Cylinder cyl_locker;

static SmemaPort smema_port;

static Sensor sens_laser1;
static Sensor sens_laser2;
static Sensor sens_laser3;
static Sensor sens_table_home;
static Sensor sens_cyl1a, sens_cyl1b;
static Sensor sens_cyl2a, sens_cyl2b;
static Sensor sens_cyl3a, sens_cyl3b;

// --- Default recipe ---
static config::Recipe default_recipe = {
    .version = config::Recipe::CURRENT_VERSION,
    .name = "default",
    .motor_presets = {
        { 10, 0 },                                       // [0] home/up rpm
        { 40, 0 },                                       // [1] convey rpm
        { 10, CONFIG_MOTOR_STEPS_PER_REV / 2 },          // [2] creep rpm + steps
        { 40, 5 * CONFIG_MOTOR_STEPS_PER_REV },           // [3] table down p1
        { 10, 1 * CONFIG_MOTOR_STEPS_PER_REV },           // [4] table down p2
        { 40, 0 },                                       // [5] convey out rpm
        { 20, 0 },                                       // [6] eject rpm
    },
    .cylinder_timeout_ms = CONFIG_CYLINDER_CONFIRM_TIMEOUT_MS,
    .laser_debounce_ms = 50,
    .crc = 0,
};

// --- SFC Sequence ---

static void interlock_check(Context &ctx, FaultManager &faults)
{
    // Poll sensors every scan
    ctx.laser1->poll();
    ctx.laser2->poll();
    ctx.laser3->poll();
    ctx.table_home->poll();
    ctx.cyl1_a->poll();
    ctx.cyl1_b->poll();
    ctx.cyl2_a->poll();
    ctx.cyl2_b->poll();
    ctx.cyl3_a->poll();
    ctx.cyl3_b->poll();

    // Check ALM signals
    if (motor_conveyor.inAlarm() || motor_width.inAlarm() || motor_table.inAlarm()) {
        faults.raise("Motor ALM asserted");
    }
}

// Sequence steps (simplified — full 32-step sequence to be expanded)
static StepDef infeed_steps[] = {
    // Step 0: Idle — wait for upstream board available
    { "idle",
      [](Context &ctx) {
          if (ctx.smema_enabled) {
              ctx.smema->setMachineReadyOut(true);
          }
      },
      nullptr,
      nullptr,
      [](Context &ctx) -> bool {
          if (ctx.smema_enabled) {
              return ctx.smema->boardAvailableIn();
          }
          return true;  // SMEMA disabled: always ready
      },
      1, nullptr, 0
    },

    // Step 1: Wait for panel presented (laser1)
    { "wait_panel",
      nullptr, nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.laser1->triggered(); },
      2, nullptr, 0
    },

    // Step 2: Convey to near position (laser2)
    { "convey_in",
      [](Context &ctx) {
          ctx.conveyor->run(ctx.recipe->motor_presets[1].rpm, MotionDir::POS);
      },
      nullptr,
      [](Context &ctx) { ctx.conveyor->stop(); },
      [](Context &ctx) -> bool { return ctx.laser2->triggered(); },
      3, nullptr, 0
    },

    // Step 3: Arm stopper
    { "arm_stopper",
      [](Context &ctx) {
          ctx.stopper->goTo(CylPosition::POS_A);
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.cyl1_a->triggered(); },
      4, nullptr, 0
    },

    // Step 4: Creep into stopper
    { "creep",
      [](Context &ctx) {
          ctx.conveyor->go(ctx.recipe->motor_presets[2].steps, ctx.recipe->motor_presets[2].rpm);
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return !ctx.conveyor->isMoving(); },
      5, nullptr, 0
    },

    // Step 5: Table down phase 1
    { "table_down_p1",
      [](Context &ctx) {
          int32_t steps = -(int32_t)ctx.recipe->motor_presets[3].steps;
          ctx.table->go(steps, ctx.recipe->motor_presets[3].rpm);
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return !ctx.table->isMoving(); },
      6, nullptr, 0
    },

    // Step 6: Table down phase 2
    { "table_down_p2",
      [](Context &ctx) {
          int32_t steps = -(int32_t)ctx.recipe->motor_presets[4].steps;
          ctx.table->go(steps, ctx.recipe->motor_presets[4].rpm);
      },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return !ctx.table->isMoving(); },
      7, nullptr, 0
    },

    // Step 7: Arm RFID
    { "arm_rfid",
      [](Context &ctx) { ctx.rfid->goTo(CylPosition::POS_A); },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.cyl2_a->triggered(); },
      8, nullptr, 0
    },

    // Step 8: Arm panel locker
    { "arm_locker",
      [](Context &ctx) { ctx.locker->goTo(CylPosition::POS_B); },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.cyl3_b->triggered(); },
      9, nullptr, 0
    },

    // Step 9: Test/Program (wait for tester result)
    { "testing",
      [](Context &ctx) { ctx.timer.start(10000); },
      nullptr, nullptr,
      [](Context &ctx) -> bool {
          // TODO: wait for tester UART result instead of timer
          return ctx.timer.expired();
      },
      10, nullptr, 0
    },

    // Step 10: Table up to home
    { "table_up",
      [](Context &ctx) {
          ctx.table->run(ctx.recipe->motor_presets[0].rpm, MotionDir::POS);
      },
      nullptr,
      [](Context &ctx) { ctx.table->stop(); },
      [](Context &ctx) -> bool { return ctx.table_home->triggered(); },
      11, nullptr, 0
    },

    // Step 11: Release RFID
    { "release_rfid",
      [](Context &ctx) { ctx.rfid->goTo(CylPosition::POS_B); },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.cyl2_b->triggered(); },
      12, nullptr, 0
    },

    // Step 12: Release locker
    { "release_locker",
      [](Context &ctx) { ctx.locker->goTo(CylPosition::POS_A); },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.cyl3_a->triggered(); },
      13, nullptr, 0
    },

    // Step 13: Disarm stopper
    { "disarm_stopper",
      [](Context &ctx) { ctx.stopper->goTo(CylPosition::POS_B); },
      nullptr, nullptr,
      [](Context &ctx) -> bool { return ctx.cyl1_b->triggered(); },
      14, nullptr, 0
    },

    // Step 14: Convey out
    { "convey_out",
      [](Context &ctx) {
          ctx.conveyor->run(ctx.recipe->motor_presets[5].rpm, MotionDir::POS);
      },
      nullptr,
      [](Context &ctx) { ctx.conveyor->stop(); },
      [](Context &ctx) -> bool { return ctx.laser3->triggered(); },
      15, nullptr, 0
    },

    // Step 15: SMEMA handoff downstream
    { "smema_out",
      [](Context &ctx) {
          if (ctx.smema_enabled) {
              ctx.smema->setBoardAvailableOut(true);
          }
      },
      nullptr,
      [](Context &ctx) {
          if (ctx.smema_enabled) {
              ctx.smema->setBoardAvailableOut(false);
          }
      },
      [](Context &ctx) -> bool {
          if (ctx.smema_enabled) {
              return ctx.smema->machineReadyIn();
          }
          return true;
      },
      16, nullptr, 0
    },

    // Step 16: Eject — run until laser3 clears
    { "eject",
      [](Context &ctx) {
          ctx.conveyor->run(ctx.recipe->motor_presets[6].rpm, MotionDir::POS);
      },
      nullptr,
      [](Context &ctx) { ctx.conveyor->stop(); },
      [](Context &ctx) -> bool { return !ctx.laser3->triggered(); },
      -1, nullptr, 0   // -1 = loop back to step 0
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

    err = motor_table.init(table_cfg);
    if (err) { LOG_ERR("table init: %d", err); return err; }

    // Cylinders
    err = cyl_stopper.init(stopper_cfg);
    if (err) { LOG_ERR("stopper init: %d", err); return err; }

    err = cyl_rfid.init(rfid_cfg);
    if (err) { LOG_ERR("rfid init: %d", err); return err; }

    err = cyl_locker.init(locker_cfg);
    if (err) { LOG_ERR("locker init: %d", err); return err; }

    // SMEMA
    err = smema_port.init(smema_cfg);
    if (err) { LOG_ERR("smema init: %d", err); return err; }

    // Sensors
    sens_laser1.init(laser1_cfg);
    sens_laser2.init(laser2_cfg);
    sens_laser3.init(laser3_cfg);
    sens_table_home.init(table_home_cfg);
    sens_cyl1a.init(cyl1a_cfg);
    sens_cyl1b.init(cyl1b_cfg);
    sens_cyl2a.init(cyl2a_cfg);
    sens_cyl2b.init(cyl2b_cfg);
    sens_cyl3a.init(cyl3a_cfg);
    sens_cyl3b.init(cyl3b_cfg);

    // Wire up context
    machine_context.conveyor = &motor_conveyor;
    machine_context.width = &motor_width;
    machine_context.table = &motor_table;
    machine_context.stopper = &cyl_stopper;
    machine_context.rfid = &cyl_rfid;
    machine_context.locker = &cyl_locker;
    machine_context.smema = &smema_port;
    machine_context.laser1 = &sens_laser1;
    machine_context.laser2 = &sens_laser2;
    machine_context.laser3 = &sens_laser3;
    machine_context.cyl1_a = &sens_cyl1a;
    machine_context.cyl1_b = &sens_cyl1b;
    machine_context.cyl2_a = &sens_cyl2a;
    machine_context.cyl2_b = &sens_cyl2b;
    machine_context.cyl3_a = &sens_cyl3a;
    machine_context.cyl3_b = &sens_cyl3b;
    machine_context.table_home = &sens_table_home;
    machine_context.recipe = &default_recipe;
    machine_context.last_result = TestResult::NONE;
    machine_context.mode = OperatingMode::AUTO;
    machine_context.cycle_count = 0;
    machine_context.advance_requested = false;
    machine_context.smema_enabled = false;   // disabled for standalone testing

    // Enable motors
    motor_conveyor.setEnabled(true);
    motor_width.setEnabled(true);
    motor_table.setEnabled(true);

    // Auto-home table at boot (moved to main.cpp after settings load)
    // if (table_cfg.auto_home) { ... }

    LOG_INF("Infeed tester machine init complete (%d steps)", INFEED_STEP_COUNT);
    return 0;
}
