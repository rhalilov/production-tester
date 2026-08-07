#include "app.h"
#include "engine/context.h"
#include "engine/engine.h"
#include "components/stepper_motor.h"
#include "components/cylinder.h"

#include <stdlib.h>
#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cli_manual, LOG_LEVEL_INF);

using namespace component;

static bool check_abort(const struct shell *sh)
{
    const struct shell_transport *transport = sh->iface;
    uint8_t c;
    size_t cnt = 0;

    if (transport->api->read(transport, &c, sizeof(c), &cnt) == 0 && cnt > 0) {
        if (c == 0x03) return true;  // Ctrl+C
    }
    return false;
}

static bool wait_motor_interruptible(const struct shell *sh, StepperMotor *motor)
{
    while (motor->isMoving()) {
        if (check_abort(sh)) {
            motor->abortMove();
            shell_warn(sh, "Aborted (Ctrl+C)");
            return false;
        }
        k_msleep(20);
    }
    return true;
}

static StepperMotor *get_motor(const struct shell *sh, const char *arg)
{
    auto *ctx = app::context();
    if (!ctx) return nullptr;

    int n = atoi(arg);
    switch (n) {
    case 1: return ctx->conveyor->beltMotor();
    case 2: return ctx->conveyor->widthMotor();
    case 3: return ctx->head->motor();
    default:
        shell_error(sh, "Motor %d not found (1-3)", n);
        return nullptr;
    }
}

static Cylinder *get_cylinder(const struct shell *sh, const char *arg)
{
    auto *ctx = app::context();
    if (!ctx) return nullptr;

    Cylinder *cyls[] = { ctx->stopper, ctx->rfid, ctx->locker };
    for (int i = 0; i < 3; i++) {
        if (strcmp(arg, cyls[i]->config().name) == 0) return cyls[i];
    }
    int n = atoi(arg);
    if (n >= 1 && n <= 3) return cyls[n - 1];
    shell_error(sh, "Unknown cylinder: %s (%s|%s|%s|1-3)", arg,
                cyls[0]->config().name, cyls[1]->config().name, cyls[2]->config().name);
    return nullptr;
}

static int cmd_motor(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    if (!eng) { shell_error(sh, "not init"); return -EINVAL; }

    if (eng->isRunning()) {
        shell_error(sh, "Sequence running — 'mc stop' first");
        return -EBUSY;
    }

    if (argc < 3) {
        shell_print(sh, "usage: manual motor <1-3|all> <run|stop|go|go_mm|home|on|off> [args]");
        shell_print(sh, "  on/off               — enable/disable driver");
        shell_print(sh, "  run <rpm> <fwd|rev>  — continuous");
        shell_print(sh, "  stop                 — stop motor");
        shell_print(sh, "  go <steps> <rpm>     — relative move");
        shell_print(sh, "  go_mm <mm> <rpm>     — relative move in mm");
        shell_print(sh, "  home                 — run homing");
        return -EINVAL;
    }

    // Handle "all" for enable/disable
    if (strcmp(argv[1], "all") == 0) {
        auto *ctx = app::context();
        if (argc >= 3 && strcmp(argv[2], "on") == 0) {
            ctx->conveyor->beltMotor()->setEnabled(true);
            ctx->conveyor->widthMotor()->setEnabled(true);
            ctx->head->motor()->setEnabled(true);
            shell_print(sh, "All motors enabled");
            return 0;
        }
        if (argc >= 3 && strcmp(argv[2], "off") == 0) {
            ctx->conveyor->beltMotor()->setEnabled(false);
            ctx->conveyor->widthMotor()->setEnabled(false);
            ctx->head->motor()->setEnabled(false);
            shell_print(sh, "All motors disabled");
            return 0;
        }
        shell_error(sh, "usage: manual motor all <on|off>");
        return -EINVAL;
    }

    auto *motor = get_motor(sh, argv[1]);
    if (!motor) return -EINVAL;

    const char *cmd = argv[2];

    if (strcmp(cmd, "on") == 0) {
        motor->setEnabled(true);
        shell_print(sh, "Motor %s enabled", argv[1]);
        return 0;
    }

    if (strcmp(cmd, "off") == 0) {
        motor->setEnabled(false);
        shell_print(sh, "Motor %s disabled", argv[1]);
        return 0;
    }

    // Auto-enable motor on any move command
    motor->setEnabled(true);

    if (strcmp(cmd, "stop") == 0) {
        motor->stop();
        shell_print(sh, "Motor %s stopped", argv[1]);
        return 0;
    }

    if (strcmp(cmd, "home") == 0) {
        shell_print(sh, "Homing motor %s... (Ctrl+C to abort)", argv[1]);
        int err = motor->home();
        if (err) {
            shell_error(sh, "Home failed: %d", err);
            return err;
        }
        shell_print(sh, "OK — homed, position=0");
        return 0;
    }

    if (strcmp(cmd, "run") == 0) {
        if (argc < 5) {
            shell_error(sh, "usage: manual motor %s run <rpm> <fwd|rev>", argv[1]);
            return -EINVAL;
        }
        uint32_t rpm = atoi(argv[3]);
        MotionDir dir = (strcmp(argv[4], "rev") == 0) ? MotionDir::NEG : MotionDir::POS;
        auto err = motor->run(rpm, dir);
        if (err != StepperMotor::Error::OK) {
            shell_error(sh, "run failed: %d", (int)err);
            return -EIO;
        }
        shell_print(sh, "Motor %s running %s @ %u rpm", argv[1], argv[4], rpm);
        return 0;
    }

    if (strcmp(cmd, "go") == 0) {
        if (argc < 5) {
            shell_error(sh, "usage: manual motor %s go <steps> <rpm>", argv[1]);
            return -EINVAL;
        }
        int32_t steps = atoi(argv[3]);
        uint32_t rpm = atoi(argv[4]);
        shell_print(sh, "Motor %s go %d steps @ %u rpm (Ctrl+C to abort)...", argv[1], steps, rpm);
        auto err = motor->startGo(steps, rpm);
        if (err != StepperMotor::Error::OK) {
            shell_error(sh, "go failed: %d", (int)err);
            return -EIO;
        }
        if (!wait_motor_interruptible(sh, motor)) {
            return -ECANCELED;
        }
        shell_print(sh, "OK — position=%d", motor->position());
        return 0;
    }

    if (strcmp(cmd, "go_mm") == 0) {
        if (argc < 5) {
            shell_error(sh, "usage: manual motor %s go_mm <mm> <rpm>", argv[1]);
            return -EINVAL;
        }
        float mm_per_rev = 0;
        int n = atoi(argv[1]);
        auto *ctx = app::context();
        if (n == 1) mm_per_rev = ctx->conveyor->config().belt_mm_per_rev;
        else if (n == 2) mm_per_rev = ctx->conveyor->config().width_mm_per_rev;
        else if (n == 3) mm_per_rev = ctx->head->config().mm_per_rev;
        if (mm_per_rev <= 0) {
            shell_error(sh, "Motor %s has no mm_per_rev configured", argv[1]);
            return -EINVAL;
        }
        float mm = strtof(argv[3], nullptr);
        int32_t steps = (int32_t)(mm * (float)motor->config().steps_per_rev / mm_per_rev);
        uint32_t rpm = atoi(argv[4]);
        shell_print(sh, "Motor %s go %.2f mm (%d steps) @ %u rpm (Ctrl+C to abort)...",
                    argv[1], (double)mm, steps, rpm);
        auto err = motor->startGo(steps, rpm);
        if (err != StepperMotor::Error::OK) {
            shell_error(sh, "go failed: %d", (int)err);
            return -EIO;
        }
        if (!wait_motor_interruptible(sh, motor)) {
            float pos_mm = (float)motor->position() * mm_per_rev / (float)motor->config().steps_per_rev;
            shell_print(sh, "Stopped at position=%d (%.2f mm)", motor->position(), (double)pos_mm);
            return -ECANCELED;
        }
        float pos_mm = (float)motor->position() * mm_per_rev / (float)motor->config().steps_per_rev;
        shell_print(sh, "OK — position=%d (%.2f mm)", motor->position(), (double)pos_mm);
        return 0;
    }

    shell_error(sh, "Unknown command: %s", cmd);
    return -EINVAL;
}

static int cmd_cylinder(const struct shell *sh, size_t argc, char **argv)
{
    auto *eng = app::engine();
    if (!eng) { shell_error(sh, "not init"); return -EINVAL; }

    if (eng->isRunning()) {
        shell_error(sh, "Sequence running — 'mc stop' first");
        return -EBUSY;
    }

    if (argc < 3) {
        shell_print(sh, "usage: manual cylinder <name|1-3> <a|b|off>");
        return -EINVAL;
    }

    auto *cyl = get_cylinder(sh, argv[1]);
    if (!cyl) return -EINVAL;

    const char *cmd = argv[2];

    if (strcmp(cmd, "off") == 0) {
        cyl->off();
        shell_print(sh, "Cylinder %s coils off", argv[1]);
        return 0;
    }

    if (strcmp(cmd, "a") == 0 || strcmp(cmd, "b") == 0) {
        CylPosition pos = (cmd[0] == 'a') ? CylPosition::POS_A : CylPosition::POS_B;
        shell_print(sh, "Cylinder %s -> pos %c ...", argv[1], cmd[0]);
        auto err = cyl->goTo(pos);
        if (err == Cylinder::Error::OK) {
            shell_print(sh, "OK — confirmed");
        } else if (err == Cylinder::Error::TIMEOUT) {
            shell_error(sh, "TIMEOUT — confirm sensor not reached");
        } else {
            shell_error(sh, "Error: %d", (int)err);
        }
        return (err == Cylinder::Error::OK) ? 0 : -EIO;
    }

    shell_error(sh, "Unknown: %s (a/b/off)", cmd);
    return -EINVAL;
}

static int cmd_smema(const struct shell *sh, size_t argc, char **argv)
{
    auto *ctx = app::context();
    if (!ctx || !ctx->conveyor) { shell_error(sh, "not init"); return -EINVAL; }

    if (argc < 2) {
        shell_print(sh, "usage: manual smema <mr_out|ba_out|ba_fail_out|off> [0|1]");
        shell_print(sh, "  mr_out <0|1>        — Machine Ready to upstream");
        shell_print(sh, "  ba_out <0|1>        — Board Available to downstream");
        shell_print(sh, "  ba_fail_out <0|1>   — Board Available NG to downstream");
        shell_print(sh, "  off                 — all outputs LOW");
        shell_print(sh, "Current state:");
        shell_print(sh, "  Up BA (in):   %s", ctx->conveyor->upstream()->boardAvailable() ? "HIGH" : "LOW");
        shell_print(sh, "  Down MR (in): %s", ctx->conveyor->downstream()->machineReady() ? "HIGH" : "LOW");
        return -EINVAL;
    }

    if (strcmp(argv[1], "off") == 0) {
        ctx->conveyor->upstream()->setMachineReady(false);
        ctx->conveyor->downstream()->allOff();
        shell_print(sh, "SMEMA all outputs LOW");
        return 0;
    }

    if (argc < 3) {
        shell_error(sh, "usage: manual smema %s <0|1>", argv[1]);
        return -EINVAL;
    }

    bool val = (atoi(argv[2]) != 0);

    if (strcmp(argv[1], "mr_out") == 0) {
        ctx->conveyor->upstream()->setMachineReady(val);
        shell_print(sh, "SMEMA MR_OUT = %s", val ? "HIGH" : "LOW");
    } else if (strcmp(argv[1], "ba_out") == 0) {
        ctx->conveyor->downstream()->setBoardAvailable(val);
        shell_print(sh, "SMEMA BA_OUT = %s", val ? "HIGH" : "LOW");
    } else if (strcmp(argv[1], "ba_fail_out") == 0) {
        ctx->conveyor->downstream()->setBoardAvailableFail(val);
        shell_print(sh, "SMEMA BA_FAIL_OUT = %s", val ? "HIGH" : "LOW");
    } else {
        shell_error(sh, "Unknown: %s (mr_out|ba_out|ba_fail_out|off)", argv[1]);
        return -EINVAL;
    }
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(manual_cmds,
    SHELL_CMD(motor, NULL, "Motor control (run/stop/go/home)", cmd_motor),
    SHELL_CMD(cylinder, NULL, "Cylinder control (a/b/off)", cmd_cylinder),
    SHELL_CMD(smema, NULL, "SMEMA output control (mr_out/ba_out/off)", cmd_smema),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(manual, &manual_cmds, "Manual actuator control", NULL);
