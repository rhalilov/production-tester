// CLI: Manual actuator commands (motor, cylinder)
// Stub — shell registration

#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cli_manual, LOG_LEVEL_INF);

static int cmd_motor(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 3) {
        shell_print(sh, "usage: motor <1-3> <run|stop|go|goto|home> [args...]");
        return -EINVAL;
    }
    shell_print(sh, "motor %s %s: TODO", argv[1], argv[2]);
    return 0;
}

static int cmd_cylinder(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 3) {
        shell_print(sh, "usage: cylinder <1-3> <a|b|off>");
        return -EINVAL;
    }
    shell_print(sh, "cylinder %s %s: TODO", argv[1], argv[2]);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(manual_cmds,
    SHELL_CMD(motor, NULL, "Motor control (run/stop/go/goto/home)", cmd_motor),
    SHELL_CMD(cylinder, NULL, "Cylinder control (a/b/off)", cmd_cylinder),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(manual, &manual_cmds, "Manual actuator control", NULL);
