#include "app.h"
#include "engine/engine.h"
#include "engine/context.h"

#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cli_machine, LOG_LEVEL_INF);

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    if (!eng) {
        shell_error(sh, "Engine not initialized");
        return -EINVAL;
    }

    if (eng->faults().inFault()) {
        shell_error(sh, "FAULT active: %s — run 'mc reset' first",
                    eng->faults().reason() ? eng->faults().reason() : "unknown");
        return -EBUSY;
    }

    eng->setMode(engine::OperatingMode::AUTO);
    eng->start();
    app::startScan();
    shell_print(sh, "OK — sequence running (AUTO mode, step[0])");
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
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(mc, &machine_cmds, "Machine control", NULL);
