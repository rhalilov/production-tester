// CLI: Configuration commands with flash persistence

#include "app.h"
#include "engine/context.h"
#include "components/stepper_motor.h"

#include <stdlib.h>
#include <string.h>
#include <zephyr/shell/shell.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cli_config, LOG_LEVEL_INF);

using namespace component;

// --- Persisted parameter block ---

struct MotorParams {
    uint32_t home_rpm;
    int32_t soft_limit_min;
    int32_t soft_limit_max;
    int32_t safe_position;
    uint32_t accel_start_rpm;
    uint32_t accel_rpm_s;
    bool invert_dir;
};

static MotorParams params[3];
static bool params_loaded;

static void apply_params(int idx, StepperMotor *motor)
{
    if (!motor) return;
    auto *cfg = const_cast<StepperMotorConfig *>(&motor->config());
    cfg->home_rpm = params[idx].home_rpm;
    cfg->soft_limit_min = params[idx].soft_limit_min;
    cfg->soft_limit_max = params[idx].soft_limit_max;
    cfg->safe_position = params[idx].safe_position;
    cfg->accel.start_rpm = params[idx].accel_start_rpm;
    cfg->accel.accel_rpm_s = params[idx].accel_rpm_s;
    cfg->invert_dir = params[idx].invert_dir;
}

static void snapshot_params(int idx, StepperMotor *motor)
{
    if (!motor) return;
    auto &c = motor->config();
    params[idx].home_rpm = c.home_rpm;
    params[idx].soft_limit_min = c.soft_limit_min;
    params[idx].soft_limit_max = c.soft_limit_max;
    params[idx].safe_position = c.safe_position;
    params[idx].accel_start_rpm = c.accel.start_rpm;
    params[idx].accel_rpm_s = c.accel.accel_rpm_s;
    params[idx].invert_dir = c.invert_dir;
}

// --- Settings handler ---

static int motor_params_set(const char *name, size_t len,
                            settings_read_cb read_cb, void *cb_arg)
{
    const char *next;

    for (int i = 0; i < 3; i++) {
        char key[2] = { (char)('1' + i), '\0' };
        if (settings_name_steq(name, key, &next) && !next) {
            if (len != sizeof(MotorParams)) return -EINVAL;
            if (read_cb(cb_arg, &params[i], sizeof(params[i])) < 0) return -EINVAL;
            params_loaded = true;
            LOG_INF("loaded motor%d params from flash", i + 1);
            return 0;
        }
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(motor_cfg, "mcfg", NULL, motor_params_set, NULL, NULL);

void config_apply_saved(void)
{
    if (!params_loaded) return;
    auto *ctx = app::context();
    if (!ctx) return;

    StepperMotor *motors[] = { ctx->conveyor, ctx->width, ctx->table };
    for (int i = 0; i < 3; i++) {
        apply_params(i, motors[i]);
    }
    LOG_INF("applied saved motor params");
}

// --- CLI commands ---

static StepperMotor *motor_by_name(const char *name, engine::Context *ctx)
{
    if (strcmp(name, "conveyor") == 0 || strcmp(name, "1") == 0) return ctx->conveyor;
    if (strcmp(name, "width") == 0 || strcmp(name, "2") == 0) return ctx->width;
    if (strcmp(name, "table") == 0 || strcmp(name, "3") == 0) return ctx->table;
    return nullptr;
}

static int motor_index(const char *name)
{
    if (strcmp(name, "conveyor") == 0 || strcmp(name, "1") == 0) return 0;
    if (strcmp(name, "width") == 0 || strcmp(name, "2") == 0) return 1;
    if (strcmp(name, "table") == 0 || strcmp(name, "3") == 0) return 2;
    return -1;
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
    auto *ctx = app::context();
    if (!ctx) { shell_error(sh, "not init"); return -EINVAL; }

    auto print_cfg = [&](const char *name, component::StepperMotor *m) {
        auto &c = m->config();
        shell_print(sh, "[%s]", name);
        shell_print(sh, "  home_rpm       = %u", c.home_rpm);
        shell_print(sh, "  invert_dir     = %s", c.invert_dir ? "true" : "false");
        shell_print(sh, "  steps_per_rev  = %u", c.steps_per_rev);
        if (c.mm_per_rev > 0) {
            shell_print(sh, "  mm_per_rev     = %.2f", (double)c.mm_per_rev);
        }
        if (c.has_limits) {
            float mm_min = c.mm_per_rev > 0 ? (float)c.soft_limit_min * c.mm_per_rev / (float)c.steps_per_rev : 0;
            float mm_max = c.mm_per_rev > 0 ? (float)c.soft_limit_max * c.mm_per_rev / (float)c.steps_per_rev : 0;
            shell_print(sh, "  soft_limit_min = %d (%.2f mm)", c.soft_limit_min, (double)mm_min);
            shell_print(sh, "  soft_limit_max = %d (%.2f mm)", c.soft_limit_max, (double)mm_max);
            shell_print(sh, "  safe_position  = %d", c.safe_position);
        }
        shell_print(sh, "  accel: start_rpm=%u accel_rpm_s=%u", c.accel.start_rpm, c.accel.accel_rpm_s);
        shell_print(sh, "");
    };

    print_cfg("1 conveyor", ctx->conveyor);
    print_cfg("2 width", ctx->width);
    print_cfg("3 table", ctx->table);

    shell_print(sh, "(%s)", params_loaded ? "loaded from flash" : "defaults — not saved yet");
    return 0;
}

static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
    auto *ctx = app::context();
    if (!ctx) { shell_error(sh, "not init"); return -EINVAL; }

    if (app::accessLevel() < app::AccessLevel::PROC_ENG) {
        shell_error(sh, "Access denied. Run 'cfg unlock proc_eng' or 'cfg unlock factory' first.");
        return -EACCES;
    }

    if (argc < 4) {
        shell_print(sh, "usage: cfg set <motor> <param> <value>");
        shell_print(sh, "  motor: 1|conveyor, 2|width, 3|table");
        shell_print(sh, "  proc_eng params: home_rpm, safe_pos, accel_start, accel_rate");
        shell_print(sh, "  factory params:  invert_dir, steps_per_rev, mm_per_rev,");
        shell_print(sh, "                   limit_min, limit_max, limit_here");
        return -EINVAL;
    }

    auto *motor = motor_by_name(argv[1], ctx);
    if (!motor) {
        shell_error(sh, "Unknown motor: %s", argv[1]);
        return -EINVAL;
    }

    const char *param = argv[2];
    const char *val_str = argv[3];
    auto *cfg = const_cast<StepperMotorConfig *>(&motor->config());

    // Factory-only parameters
    bool is_factory_param = (strcmp(param, "invert_dir") == 0 ||
                             strcmp(param, "steps_per_rev") == 0 ||
                             strcmp(param, "mm_per_rev") == 0 ||
                             strcmp(param, "limit_min") == 0 ||
                             strcmp(param, "limit_max") == 0 ||
                             strcmp(param, "limit_here") == 0);

    if (is_factory_param && !app::isFactory()) {
        shell_error(sh, "'%s' is factory-level. Run 'cfg unlock factory' first.", param);
        return -EACCES;
    }

    if (strcmp(param, "home_rpm") == 0) {
        cfg->home_rpm = atoi(val_str);
        shell_print(sh, "home_rpm = %u", cfg->home_rpm);
    } else if (strcmp(param, "invert_dir") == 0) {
        cfg->invert_dir = (atoi(val_str) != 0 || strcmp(val_str, "true") == 0);
        shell_print(sh, "invert_dir = %s", cfg->invert_dir ? "true" : "false");
    } else if (strcmp(param, "steps_per_rev") == 0) {
        cfg->steps_per_rev = atoi(val_str);
        shell_print(sh, "steps_per_rev = %u", cfg->steps_per_rev);
    } else if (strcmp(param, "mm_per_rev") == 0) {
        cfg->mm_per_rev = strtof(val_str, nullptr);
        shell_print(sh, "mm_per_rev = %.2f", (double)cfg->mm_per_rev);
    } else if (strcmp(param, "limit_min") == 0) {
        float mm_per_step = cfg->mm_per_rev > 0 ? cfg->mm_per_rev / (float)cfg->steps_per_rev : 0;
        if (mm_per_step > 0) {
            float mm = strtof(val_str, nullptr);
            cfg->soft_limit_min = (int32_t)(mm / mm_per_step);
            shell_print(sh, "soft_limit_min = %.2f mm (%d steps)", (double)mm, cfg->soft_limit_min);
        } else {
            cfg->soft_limit_min = atoi(val_str);
            shell_print(sh, "soft_limit_min = %d steps", cfg->soft_limit_min);
        }
    } else if (strcmp(param, "limit_max") == 0) {
        float mm_per_step = cfg->mm_per_rev > 0 ? cfg->mm_per_rev / (float)cfg->steps_per_rev : 0;
        if (mm_per_step > 0) {
            float mm = strtof(val_str, nullptr);
            cfg->soft_limit_max = (int32_t)(mm / mm_per_step);
            shell_print(sh, "soft_limit_max = %.2f mm (%d steps)", (double)mm, cfg->soft_limit_max);
        } else {
            cfg->soft_limit_max = atoi(val_str);
            shell_print(sh, "soft_limit_max = %d steps", cfg->soft_limit_max);
        }
    } else if (strcmp(param, "limit_here") == 0) {
        int32_t pos = motor->position();
        float mm_per_step = cfg->mm_per_rev > 0 ? cfg->mm_per_rev / (float)cfg->steps_per_rev : 0;
        float mm = mm_per_step > 0 ? (float)pos * mm_per_step : 0;
        if (strcmp(val_str, "min") == 0) {
            cfg->soft_limit_min = pos;
            shell_print(sh, "soft_limit_min = %.2f mm (%d steps) [current pos]", (double)mm, pos);
        } else if (strcmp(val_str, "max") == 0) {
            cfg->soft_limit_max = pos;
            shell_print(sh, "soft_limit_max = %.2f mm (%d steps) [current pos]", (double)mm, pos);
        } else {
            shell_error(sh, "usage: cfg set %s limit_here <min|max>", argv[1]);
            return -EINVAL;
        }
    } else if (strcmp(param, "safe_pos") == 0) {
        cfg->safe_position = atoi(val_str);
        shell_print(sh, "safe_position = %d", cfg->safe_position);
    } else if (strcmp(param, "accel_start") == 0) {
        cfg->accel.start_rpm = atoi(val_str);
        shell_print(sh, "accel.start_rpm = %u", cfg->accel.start_rpm);
    } else if (strcmp(param, "accel_rate") == 0) {
        cfg->accel.accel_rpm_s = atoi(val_str);
        shell_print(sh, "accel.accel_rpm_s = %u", cfg->accel.accel_rpm_s);
    } else {
        shell_error(sh, "Unknown param: %s", param);
        return -EINVAL;
    }

    return 0;
}

static int cmd_save(const struct shell *sh, size_t argc, char **argv)
{
    auto *ctx = app::context();
    if (!ctx) { shell_error(sh, "not init"); return -EINVAL; }

    if (app::accessLevel() < app::AccessLevel::PROC_ENG) {
        shell_error(sh, "Access denied. Run 'cfg unlock proc_eng' first.");
        return -EACCES;
    }

    StepperMotor *motors[] = { ctx->conveyor, ctx->width, ctx->table };
    for (int i = 0; i < 3; i++) {
        snapshot_params(i, motors[i]);
        char key[8];
        snprintf(key, sizeof(key), "mcfg/%d", i + 1);
        int rc = settings_save_one(key, &params[i], sizeof(params[i]));
        if (rc) {
            shell_error(sh, "Failed to save motor %d: %d", i + 1, rc);
            return rc;
        }
    }

    shell_print(sh, "All motor params saved to flash.");
    return 0;
}

static int cmd_limit(const struct shell *sh, size_t argc, char **argv)
{
    auto *ctx = app::context();
    if (!ctx) { shell_error(sh, "not init"); return -EINVAL; }

    if (argc < 2) {
        shell_print(sh, "usage: cfg limit <motor> [min|max|here] [mm]");
        shell_print(sh, "  motor: 1|conveyor, 2|width, 3|table");
        shell_print(sh, "  cfg limit 3            — show limits");
        shell_print(sh, "  cfg limit 3 min -75    — set min to -75 mm");
        shell_print(sh, "  cfg limit 3 max 0      — set max to 0 mm");
        shell_print(sh, "  cfg limit 3 here min   — set min to current pos");
        shell_print(sh, "  cfg limit 3 here max   — set max to current pos");
        return -EINVAL;
    }

    auto *motor = motor_by_name(argv[1], ctx);
    if (!motor) {
        shell_error(sh, "Unknown motor: %s", argv[1]);
        return -EINVAL;
    }

    auto &c = motor->config();
    float mm_per_step = (c.mm_per_rev > 0) ? c.mm_per_rev / (float)c.steps_per_rev : 0;

    if (argc < 3) {
        float mm_min = mm_per_step > 0 ? (float)c.soft_limit_min * mm_per_step : 0;
        float mm_max = mm_per_step > 0 ? (float)c.soft_limit_max * mm_per_step : 0;
        float mm_pos = mm_per_step > 0 ? (float)motor->position() * mm_per_step : 0;
        shell_print(sh, "Motor %s limits:", argv[1]);
        if (mm_per_step > 0) {
            shell_print(sh, "  min = %.2f mm (%d steps)", (double)mm_min, c.soft_limit_min);
            shell_print(sh, "  max = %.2f mm (%d steps)", (double)mm_max, c.soft_limit_max);
            shell_print(sh, "  pos = %.2f mm (%d steps)", (double)mm_pos, motor->position());
        } else {
            shell_print(sh, "  min = %d steps", c.soft_limit_min);
            shell_print(sh, "  max = %d steps", c.soft_limit_max);
            shell_print(sh, "  pos = %d steps", motor->position());
        }
        return 0;
    }

    if (strcmp(argv[2], "here") == 0) {
        int32_t pos = motor->position();
        float mm = mm_per_step > 0 ? (float)pos * mm_per_step : 0;
        const char *which = (argc >= 4) ? argv[3] : "min";
        if (strcmp(which, "min") == 0) {
            motor->setSoftLimits(pos, c.soft_limit_max);
            if (mm_per_step > 0)
                shell_print(sh, "soft_limit_min = %.2f mm (%d steps)", (double)mm, pos);
            else
                shell_print(sh, "soft_limit_min = %d steps", pos);
        } else if (strcmp(which, "max") == 0) {
            motor->setSoftLimits(c.soft_limit_min, pos);
            if (mm_per_step > 0)
                shell_print(sh, "soft_limit_max = %.2f mm (%d steps)", (double)mm, pos);
            else
                shell_print(sh, "soft_limit_max = %d steps", pos);
        } else {
            shell_error(sh, "usage: cfg limit %s here <min|max>", argv[1]);
            return -EINVAL;
        }
        return 0;
    }

    if (argc < 4) {
        shell_error(sh, "usage: cfg limit %s <min|max> <mm>", argv[1]);
        return -EINVAL;
    }

    float mm_val = strtof(argv[3], nullptr);
    int32_t steps;
    if (mm_per_step > 0) {
        steps = (int32_t)(mm_val / mm_per_step);
    } else {
        steps = (int32_t)mm_val;
    }

    if (strcmp(argv[2], "min") == 0) {
        motor->setSoftLimits(steps, c.soft_limit_max);
        shell_print(sh, "soft_limit_min = %.2f mm (%d steps)", (double)mm_val, steps);
    } else if (strcmp(argv[2], "max") == 0) {
        motor->setSoftLimits(c.soft_limit_min, steps);
        shell_print(sh, "soft_limit_max = %.2f mm (%d steps)", (double)mm_val, steps);
    } else {
        shell_error(sh, "Unknown: %s (min/max/here)", argv[2]);
        return -EINVAL;
    }
    return 0;
}

static int cmd_unlock(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "usage: cfg unlock <operator|proc_eng|factory>");
        shell_print(sh, "Current level: %s",
            app::accessLevel() == app::AccessLevel::FACTORY ? "FACTORY" :
            app::accessLevel() == app::AccessLevel::PROC_ENG ? "proc_eng" : "operator");
        return -EINVAL;
    }

    if (strcmp(argv[1], "operator") == 0) {
        app::setAccessLevel(app::AccessLevel::OPERATOR);
        shell_print(sh, "Access: OPERATOR (view only, start/stop)");
    } else if (strcmp(argv[1], "proc_eng") == 0) {
        app::setAccessLevel(app::AccessLevel::PROC_ENG);
        shell_print(sh, "Access: PROC_ENG (can change speeds, positions, limits)");
    } else if (strcmp(argv[1], "factory") == 0) {
        app::setAccessLevel(app::AccessLevel::FACTORY);
        shell_print(sh, "Access: FACTORY (full access, soft limits DISABLED)");
    } else {
        shell_error(sh, "Unknown level: %s", argv[1]);
        return -EINVAL;
    }
    return 0;
}

static int cmd_lock(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    app::setAccessLevel(app::AccessLevel::OPERATOR);
    shell_print(sh, "Locked. Access: OPERATOR");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(config_cmds,
    SHELL_CMD(show, NULL, "Show all motor parameters", cmd_show),
    SHELL_CMD(set, NULL, "Set param: cfg set <motor> <param> <value>", cmd_set),
    SHELL_CMD(save, NULL, "Save params to flash", cmd_save),
    SHELL_CMD(limit, NULL, "Soft limits per motor", cmd_limit),
    SHELL_CMD(unlock, NULL, "Unlock access: cfg unlock <operator|proc_eng|factory>", cmd_unlock),
    SHELL_CMD(lock, NULL, "Lock to operator level", cmd_lock),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(cfg, &config_cmds, "Configuration", NULL);
