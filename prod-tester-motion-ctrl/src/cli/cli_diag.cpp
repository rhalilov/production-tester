// CLI: Diagnostics commands (status, sensors, version, uptime)
// Stub — shell registration

#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cli_diag, LOG_LEVEL_INF);

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "status: TODO");
    return 0;
}

static int cmd_sensors(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "sensors: TODO");
    return 0;
}

static int cmd_version(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "prod-tester-motion-ctrl v0.1.0");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(diag_cmds,
    SHELL_CMD(status, NULL, "Machine state, step, mode, faults", cmd_status),
    SHELL_CMD(sensors, NULL, "All sensor states", cmd_sensors),
    SHELL_CMD(version, NULL, "Firmware version", cmd_version),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(diag, &diag_cmds, "Diagnostics", NULL);
