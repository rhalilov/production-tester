#include "app.h"
#include "engine/engine.h"
#include "engine/context.h"
#include "components/stepper_motor.h"
#include "components/cylinder.h"
#include "components/sensor.h"

#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cli_machine, LOG_LEVEL_INF);

static int detect_start_step(engine::Context *ctx)
{
    ctx->laser1->poll();
    ctx->laser2->poll();
    ctx->laser3->poll();

    // laser2 has priority — board is at work position
    if (ctx->laser2->raw()) {
        return 3;   // arm_stopper (board at work position)
    }
    if (ctx->laser3->raw()) {
        return 16;  // eject (board at exit)
    }
    if (ctx->laser1->raw()) {
        return 1;   // wait_panel (board at infeed)
    }
    return 0;       // idle
}

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    auto *ctx = app::context();
    if (!eng || !ctx) {
        shell_error(sh, "Engine not initialized");
        return -EINVAL;
    }

    if (eng->faults().inFault()) {
        shell_error(sh, "FAULT active: %s — run 'mc reset' first",
                    eng->faults().reason() ? eng->faults().reason() : "unknown");
        return -EBUSY;
    }

    int start_step = detect_start_step(ctx);

    ctx->conveyor->setEnabled(true);
    ctx->table->setEnabled(true);

    eng->setMode(engine::OperatingMode::AUTO);
    eng->start();

    if (start_step > 0) {
        eng->gotoStep(start_step);
    }

    app::startScan();
    shell_print(sh, "OK — sequence running (AUTO mode, step[%d] \"%s\")",
                start_step, eng->config().steps[start_step].name);
    return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    auto *ctx = app::context();
    if (!eng) { shell_error(sh, "not init"); return -EINVAL; }

    app::stopScan();
    eng->stop();

    ctx->conveyor->stop();
    ctx->width->stop();
    ctx->table->stop();
    ctx->stopper->off();
    ctx->rfid->off();
    ctx->locker->off();

    shell_print(sh, "OK — stopped, all outputs off");
    return 0;
}

static int cmd_abort(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    auto *ctx = app::context();
    if (!eng) { shell_error(sh, "not init"); return -EINVAL; }

    app::stopScan();
    eng->stop();
    eng->faults().raise("operator abort");

    ctx->conveyor->emergencyStop();
    ctx->width->emergencyStop();
    ctx->table->emergencyStop();
    ctx->stopper->off();
    ctx->rfid->off();
    ctx->locker->off();
    ctx->smema->allOff();

    shell_print(sh, "ABORTED — all motion stopped, fault latched");
    return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    auto *ctx = app::context();
    if (!eng) { shell_error(sh, "not init"); return -EINVAL; }

    eng->faults().reset();
    ctx->conveyor->clearAlarm();
    ctx->width->clearAlarm();
    ctx->table->clearAlarm();

    shell_print(sh, "OK — fault cleared. Run 'mc start' to resume.");
    return 0;
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    if (!eng) { shell_error(sh, "not init"); return -EINVAL; }

    if (argc < 2) {
        const char *names[] = { "AUTO", "STEP", "MANUAL" };
        shell_print(sh, "Current mode: %s", names[(int)eng->mode()]);
        return 0;
    }

    if (strcmp(argv[1], "auto") == 0) {
        eng->setMode(engine::OperatingMode::AUTO);
    } else if (strcmp(argv[1], "step") == 0) {
        eng->setMode(engine::OperatingMode::STEP);
    } else if (strcmp(argv[1], "manual") == 0) {
        app::stopScan();
        eng->stop();
        eng->setMode(engine::OperatingMode::MANUAL);
        shell_print(sh, "MANUAL mode — use 'manual motor/cylinder' commands");
        return 0;
    } else {
        shell_error(sh, "Unknown mode: %s (auto/step/manual)", argv[1]);
        return -EINVAL;
    }

    shell_print(sh, "OK — mode: %s", argv[1]);
    return 0;
}

static int cmd_step_next(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    if (!eng) { shell_error(sh, "not init"); return -EINVAL; }

    if (eng->mode() != engine::OperatingMode::STEP) {
        shell_error(sh, "Not in STEP mode — run 'mc mode step' first");
        return -EINVAL;
    }

    eng->advanceStep();
    shell_print(sh, "OK — advance requested");
    return 0;
}

static int cmd_step_current(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    if (!eng) { shell_error(sh, "not init"); return -EINVAL; }

    auto *step = eng->currentStep();
    shell_print(sh, "Step[%d]: \"%s\" (running:%s fault:%s)",
                eng->currentStepIndex(),
                step ? step->name : "none",
                eng->isRunning() ? "yes" : "no",
                eng->faults().inFault() ? "yes" : "no");
    return 0;
}

static int cmd_step_list(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    if (!eng) { shell_error(sh, "not init"); return -EINVAL; }

    int count = eng->config().step_count;
    int cur = eng->currentStepIndex();

    shell_print(sh, "=== Sequence (%d steps) ===", count);
    for (int i = 0; i < count; i++) {
        const char *marker = (i == cur && eng->isRunning()) ? " <<" : "";
        shell_print(sh, "  [%2d] %s%s",
                    i, eng->config().steps[i].name, marker);
    }
    return 0;
}

// --- High-level machine operations ---

#define POLL_MS 5

static bool wait_sensor(component::Sensor *s, bool target, uint32_t timeout_ms)
{
    uint32_t waited = 0;
    while (true) {
        s->poll();
        if (s->raw() == target) return true;
        if (timeout_ms > 0 && waited >= timeout_ms) return false;
        k_msleep(POLL_MS);
        waited += POLL_MS;
    }
}

static int cmd_load(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    auto *ctx = app::context();
    if (!eng || !ctx) { shell_error(sh, "not init"); return -EINVAL; }

    if (eng->isRunning()) {
        shell_error(sh, "Sequence running — 'mc stop' first");
        return -EBUSY;
    }

    ctx->laser2->poll();
    if (ctx->laser2->raw()) {
        shell_print(sh, "Panel already at position (laser2 active)");
        return 0;
    }

    ctx->conveyor->setEnabled(true);

    shell_print(sh, "Waiting for panel at infeed (laser1)...");
    if (!wait_sensor(ctx->laser1, true, 0)) return -EINTR;
    shell_print(sh, "Panel detected. Conveyor running...");

    ctx->conveyor->run(30, component::MotionDir::POS);

    shell_print(sh, "Waiting for panel near (laser2)...");
    if (!wait_sensor(ctx->laser2, true, 30000)) {
        ctx->conveyor->stop();
        shell_error(sh, "Timeout waiting for laser2");
        return -ETIMEDOUT;
    }
    ctx->conveyor->stop();
    shell_print(sh, "OK — panel loaded (at laser2)");
    return 0;
}

static bool wait_motor_done(component::StepperMotor *m, uint32_t timeout_ms)
{
    uint32_t waited = 0;
    while (m->isMoving()) {
        k_msleep(POLL_MS);
        waited += POLL_MS;
        if (timeout_ms > 0 && waited >= timeout_ms) return false;
    }
    return true;
}

static int cmd_unload(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    auto *ctx = app::context();
    if (!eng || !ctx) { shell_error(sh, "not init"); return -EINVAL; }

    if (eng->isRunning()) {
        shell_error(sh, "Sequence running — 'mc stop' first");
        return -EBUSY;
    }

    if (!ctx->table->isHomed()) {
        shell_error(sh, "Table not homed — run 'mc home' first");
        return -EINVAL;
    }

    const auto &tpos = ctx->recipe->table_pos;
    int32_t pos_pins_touch = ctx->table->mmToSteps(tpos.pins_touch);
    int32_t pos_guides_clear = ctx->table->mmToSteps(tpos.guides_clear);
    int32_t cur_pos = ctx->table->position();
    uint32_t up_rpm = ctx->recipe->motor_presets[0].rpm;

    ctx->table->setEnabled(true);
    ctx->conveyor->setEnabled(true);

    // Phase 1: If below pins_touch (at contact), raise to pins_touch
    if (cur_pos < pos_pins_touch) {
        shell_print(sh, "Raising to pins_touch...");
        ctx->table->goTo(pos_pins_touch, ctx->recipe->motor_presets[4].rpm);
        if (!wait_motor_done(ctx->table, 15000)) {
            shell_error(sh, "Timeout raising to pins_touch");
            return -ETIMEDOUT;
        }
    }

    // Phase 2: Retract RFID (if armed)
    if (ctx->rfid->currentPos() == component::CylPosition::POS_A) {
        shell_print(sh, "Retracting RFID...");
        ctx->rfid->goTo(component::CylPosition::POS_B);
    }

    // Phase 3: Raise to guides_clear
    cur_pos = ctx->table->position();
    if (cur_pos < pos_guides_clear) {
        shell_print(sh, "Raising to guides_clear...");
        ctx->table->goTo(pos_guides_clear, up_rpm);
        if (!wait_motor_done(ctx->table, 15000)) {
            shell_error(sh, "Timeout raising to guides_clear");
            return -ETIMEDOUT;
        }
    }

    // Phase 4: Release locker
    if (ctx->locker->currentPos() == component::CylPosition::POS_B) {
        shell_print(sh, "Releasing locker...");
        ctx->locker->goTo(component::CylPosition::POS_A);
    }

    // Phase 5: Retract stopper
    if (ctx->stopper->currentPos() == component::CylPosition::POS_A) {
        shell_print(sh, "Retracting stopper...");
        ctx->stopper->goTo(component::CylPosition::POS_B);
    }

    // Phase 6: Convey out + eject
    shell_print(sh, "Conveying out...");
    ctx->conveyor->run(ctx->recipe->motor_presets[5].rpm, component::MotionDir::POS);

    wait_sensor(ctx->laser3, true, 5000);
    if (!wait_sensor(ctx->laser3, false, 30000)) {
        ctx->conveyor->stop();
        shell_error(sh, "Timeout — panel didn't clear laser3");
        return -ETIMEDOUT;
    }
    ctx->conveyor->stop();
    shell_print(sh, "OK — panel ejected");
    return 0;
}

static int cmd_eject(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    auto *ctx = app::context();
    if (!eng || !ctx) { shell_error(sh, "not init"); return -EINVAL; }

    if (eng->isRunning()) {
        shell_error(sh, "Sequence running — 'mc stop' first");
        return -EBUSY;
    }

    ctx->conveyor->setEnabled(true);
    ctx->conveyor->run(40, component::MotionDir::POS);
    shell_print(sh, "Ejecting panel. Waiting for laser3 to open...");

    // Wait for laser3 to be triggered first (panel there)
    wait_sensor(ctx->laser3, true, 5000);
    // Now wait for it to open (panel passed)
    if (!wait_sensor(ctx->laser3, false, 30000)) {
        ctx->conveyor->stop();
        shell_error(sh, "Timeout — panel didn't clear laser3");
        return -ETIMEDOUT;
    }
    ctx->conveyor->stop();
    shell_print(sh, "OK — panel ejected");
    return 0;
}

static int cmd_home(const struct shell *sh, size_t argc, char **argv)
{
    auto *ctx = app::context();
    auto *eng = app::engine();
    if (!eng || !ctx) { shell_error(sh, "not init"); return -EINVAL; }

    if (eng->isRunning()) {
        shell_error(sh, "Sequence running — 'mc stop' first");
        return -EBUSY;
    }

    ctx->table->setEnabled(true);
    shell_print(sh, "Homing table...");
    int err = ctx->table->home();
    if (err) {
        shell_error(sh, "Home failed: %d", err);
        return err;
    }
    shell_print(sh, "OK — table homed (position=0)");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(step_cmds,
    SHELL_CMD(next, NULL, "Advance one step (STEP mode)", cmd_step_next),
    SHELL_CMD(current, NULL, "Show current step", cmd_step_current),
    SHELL_CMD(list, NULL, "List all steps in sequence", cmd_step_list),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(machine_cmds,
    SHELL_CMD(start, NULL, "Start sequence (AUTO mode)", cmd_start),
    SHELL_CMD(stop, NULL, "Stop sequence, outputs off", cmd_stop),
    SHELL_CMD(abort, NULL, "Emergency stop + fault latch", cmd_abort),
    SHELL_CMD(reset, NULL, "Clear fault", cmd_reset),
    SHELL_CMD(mode, NULL, "Get/set mode (auto/step/manual)", cmd_mode),
    SHELL_CMD(step, &step_cmds, "Step control (next/current)", NULL),
    SHELL_CMD(load, NULL, "Load panel (conveyor to laser2)", cmd_load),
    SHELL_CMD(unload, NULL, "Unload panel (conveyor to laser3)", cmd_unload),
    SHELL_CMD(eject, NULL, "Eject panel (run until laser3 clears)", cmd_eject),
    SHELL_CMD(home, NULL, "Home table motor", cmd_home),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(mc, &machine_cmds, "Machine control", NULL);
