// CLI: Machine control commands (start, stop, abort, reset, mode)
// Stub — shell registration

#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cli_machine, LOG_LEVEL_INF);

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "start: TODO");
    return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "stop: TODO");
    return 0;
}

static int cmd_abort(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "abort: TODO");
    return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "reset: TODO");
    return 0;
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "usage: mode <auto|step|manual>");
        return -EINVAL;
    }
    shell_print(sh, "mode %s: TODO", argv[1]);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(machine_cmds,
    SHELL_CMD(start, NULL, "Start/resume auto cycle", cmd_start),
    SHELL_CMD(stop, NULL, "Pause sequence (manual mode)", cmd_stop),
    SHELL_CMD(abort, NULL, "Emergency stop", cmd_abort),
    SHELL_CMD(reset, NULL, "Clear fault, re-home", cmd_reset),
    SHELL_CMD(mode, NULL, "Set operating mode (auto/step/manual)", cmd_mode),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(mc, &machine_cmds, "Machine control", NULL);
