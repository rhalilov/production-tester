// CLI: Configuration commands (recipe, set, cal, param)
// Stub — shell registration

#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cli_config, LOG_LEVEL_INF);

static int cmd_recipe(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "usage: recipe <list|load|save|delete|show>");
        return -EINVAL;
    }
    shell_print(sh, "recipe %s: TODO", argv[1]);
    return 0;
}

static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 3) {
        shell_print(sh, "usage: set <param> <value>");
        return -EINVAL;
    }
    shell_print(sh, "set %s = %s: TODO", argv[1], argv[2]);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(config_cmds,
    SHELL_CMD(recipe, NULL, "Recipe management (list/load/save/delete/show)", cmd_recipe),
    SHELL_CMD(set, NULL, "Set operator parameter", cmd_set),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(cfg, &config_cmds, "Configuration", NULL);
