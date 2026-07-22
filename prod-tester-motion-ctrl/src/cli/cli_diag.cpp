#include "app.h"
#include "engine/engine.h"
#include "engine/context.h"
#include "components/stepper_motor.h"
#include "components/cylinder.h"
#include "components/sensor.h"

#include <zephyr/shell/shell.h>
#include <zephyr/version.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cli_diag, LOG_LEVEL_INF);

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    auto *ctx = app::context();
    if (!eng || !ctx) { shell_error(sh, "not init"); return -EINVAL; }

    const char *modes[] = { "AUTO", "STEP", "MANUAL" };
    const char *fstates[] = { "NORMAL", "FAULT", "RECOVERY" };
    auto *step = eng->currentStep();

    shell_print(sh, "=== Machine Status ===");
    shell_print(sh, "Mode:    %s", modes[(int)eng->mode()]);
    shell_print(sh, "Running: %s", eng->isRunning() ? "YES" : "no");
    shell_print(sh, "Fault:   %s%s%s",
                fstates[(int)eng->faults().state()],
                eng->faults().reason() ? " (" : "",
                eng->faults().reason() ? eng->faults().reason() : "");
    if (eng->faults().reason()) shell_print(sh, ")");
    shell_print(sh, "Step:    [%d] %s", eng->currentStepIndex(),
                step ? step->name : "none");
    shell_print(sh, "Cycles:  %u", eng->cycleCount());
    shell_print(sh, "");
    shell_print(sh, "=== Motors ===");
    shell_print(sh, "  Conveyor: pos=%d moving=%s homed=%s alm=%s",
                ctx->conveyor->position(),
                ctx->conveyor->isMoving() ? "YES" : "no",
                ctx->conveyor->isHomed() ? "yes" : "n/a",
                ctx->conveyor->inAlarm() ? "ALM!" : "ok");
    shell_print(sh, "  Width:    pos=%d moving=%s homed=%s alm=%s",
                ctx->width->position(),
                ctx->width->isMoving() ? "YES" : "no",
                ctx->width->isHomed() ? "yes" : "no",
                ctx->width->inAlarm() ? "ALM!" : "ok");
    shell_print(sh, "  Table:    pos=%d moving=%s homed=%s alm=%s",
                ctx->table->position(),
                ctx->table->isMoving() ? "YES" : "no",
                ctx->table->isHomed() ? "yes" : "no",
                ctx->table->inAlarm() ? "ALM!" : "ok");
    shell_print(sh, "");
    shell_print(sh, "=== Cylinders ===");
    const char *posnames[] = { "A", "B", "?" };
    shell_print(sh, "  Stopper: %s", posnames[(int)ctx->stopper->currentPos()]);
    shell_print(sh, "  RFID:    %s", posnames[(int)ctx->rfid->currentPos()]);
    shell_print(sh, "  Locker:  %s", posnames[(int)ctx->locker->currentPos()]);

    return 0;
}

static int cmd_sensors(const struct shell *sh, size_t argc, char **argv)
{
    auto *ctx = app::context();
    if (!ctx) { shell_error(sh, "not init"); return -EINVAL; }

    // Force a fresh physical read by polling
    component::Sensor *all[] = {
        ctx->laser1, ctx->laser2, ctx->laser3, ctx->table_home,
        ctx->cyl1_a, ctx->cyl1_b, ctx->cyl2_a, ctx->cyl2_b,
        ctx->cyl3_a, ctx->cyl3_b
    };
    for (auto *s : all) { s->poll(); }

    shell_print(sh, "=== Sensors (raw pin state) ===");
    shell_print(sh, "%-12s %s", "laser1", ctx->laser1->raw() ? "CLOSED" : "open");
    shell_print(sh, "%-12s %s", "laser2", ctx->laser2->raw() ? "CLOSED" : "open");
    shell_print(sh, "%-12s %s", "laser3", ctx->laser3->raw() ? "CLOSED" : "open");
    shell_print(sh, "%-12s %s", "table_home", ctx->table_home->raw() ? "CLOSED" : "open");
    shell_print(sh, "");
    shell_print(sh, "=== Cylinder Confirms ===");
    shell_print(sh, "%-12s %s", "cyl1_a", ctx->cyl1_a->raw() ? "CLOSED" : "open");
    shell_print(sh, "%-12s %s", "cyl1_b", ctx->cyl1_b->raw() ? "CLOSED" : "open");
    shell_print(sh, "%-12s %s", "cyl2_a", ctx->cyl2_a->raw() ? "CLOSED" : "open");
    shell_print(sh, "%-12s %s", "cyl2_b", ctx->cyl2_b->raw() ? "CLOSED" : "open");
    shell_print(sh, "%-12s %s", "cyl3_a", ctx->cyl3_a->raw() ? "CLOSED" : "open");
    shell_print(sh, "%-12s %s", "cyl3_b", ctx->cyl3_b->raw() ? "CLOSED" : "open");
    shell_print(sh, "");
    shell_print(sh, "=== Motor ALM ===");
    shell_print(sh, "  Conveyor: %s", ctx->conveyor->inAlarm() ? "ALM!" : "ok");
    shell_print(sh, "  Width:    %s", ctx->width->inAlarm() ? "ALM!" : "ok");
    shell_print(sh, "  Table:    %s", ctx->table->inAlarm() ? "ALM!" : "ok");
    shell_print(sh, "");
    shell_print(sh, "=== SMEMA ===");
    shell_print(sh, "  Up BA (in):  %s", ctx->smema->boardAvailableIn() ? "HIGH" : "low");
    shell_print(sh, "  Down MR (in): %s", ctx->smema->machineReadyIn() ? "HIGH" : "low");

    return 0;
}

static int cmd_version(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "prod-tester-motion-ctrl v0.1.0");
    shell_print(sh, "Machine: Infeed PCB tester (Shelly)");
    shell_print(sh, "MCU: STM32F407 / Zephyr %s", KERNEL_VERSION_STRING);
    return 0;
}

static int cmd_pos(const struct shell *sh, size_t argc, char **argv)
{
    auto *ctx = app::context();
    if (!ctx) { shell_error(sh, "not init"); return -EINVAL; }

    shell_print(sh, "=== Motor Positions ===");

    auto print_motor = [&](const char *name, component::StepperMotor *m) {
        float mm_rev = m->config().mm_per_rev;
        if (mm_rev > 0) {
            float mm = (float)m->position() * mm_rev / (float)m->config().steps_per_rev;
            shell_print(sh, "  %s: %d steps (%.2f mm) %s", name, m->position(), (double)mm,
                        m->isHomed() ? "" : "[NOT HOMED]");
        } else {
            shell_print(sh, "  %s: %d steps %s", name, m->position(),
                        m->isHomed() ? "" : "[NOT HOMED]");
        }
    };

    print_motor("1 Conveyor", ctx->conveyor);
    print_motor("2 Width", ctx->width);
    print_motor("3 Table", ctx->table);

    shell_print(sh, "");
    float lim_mm = (float)ctx->table->config().soft_limit_min * 5.0f / (float)ctx->table->config().steps_per_rev;
    shell_print(sh, "  Table soft limits: min=%d (%.2f mm) max=%d",
                ctx->table->config().soft_limit_min, (double)lim_mm,
                ctx->table->config().soft_limit_max);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(diag_cmds,
    SHELL_CMD(status, NULL, "Machine state overview", cmd_status),
    SHELL_CMD(sensors, NULL, "All sensor states", cmd_sensors),
    SHELL_CMD(pos, NULL, "Motor positions", cmd_pos),
    SHELL_CMD(version, NULL, "Firmware version", cmd_version),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(diag, &diag_cmds, "Diagnostics", NULL);
