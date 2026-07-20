#include "config_store.h"

#include <string.h>
#include <errno.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(config_store, LOG_LEVEL_INF);

namespace config {

static FactoryConfig s_factory;
static Recipe s_recipes[MAX_RECIPES];
static int s_active_slot;
static uint32_t s_cycle_count;
static int32_t s_positions[3];

static int settings_set_cb(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
    const char *next;

    if (settings_name_steq(name, "factory", &next) && !next) {
        if (len == sizeof(FactoryConfig)) {
            read_cb(cb_arg, &s_factory, sizeof(s_factory));
        }
        return 0;
    }

    if (settings_name_steq(name, "active", &next) && !next) {
        if (len == sizeof(int)) {
            read_cb(cb_arg, &s_active_slot, sizeof(s_active_slot));
        }
        return 0;
    }

    if (settings_name_steq(name, "cycles", &next) && !next) {
        if (len == sizeof(uint32_t)) {
            read_cb(cb_arg, &s_cycle_count, sizeof(s_cycle_count));
        }
        return 0;
    }

    // recipe/0 .. recipe/7
    if (settings_name_steq(name, "r", &next) && next) {
        int slot = next[0] - '0';
        if (slot >= 0 && slot < MAX_RECIPES && len == sizeof(Recipe)) {
            read_cb(cb_arg, &s_recipes[slot], sizeof(Recipe));
        }
        return 0;
    }

    // pos/0 .. pos/2
    if (settings_name_steq(name, "pos", &next) && next) {
        int axis = next[0] - '0';
        if (axis >= 0 && axis < 3 && len == sizeof(int32_t)) {
            read_cb(cb_arg, &s_positions[axis], sizeof(int32_t));
        }
        return 0;
    }

    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(ptmc, "ptmc", NULL, settings_set_cb, NULL, NULL);

int ConfigStore::init()
{
    active_slot_ = 0;
    memset(&s_factory, 0, sizeof(s_factory));
    memset(s_recipes, 0, sizeof(s_recipes));
    s_cycle_count = 0;
    memset(s_positions, 0, sizeof(s_positions));

    int err = settings_subsys_init();
    if (err) {
        LOG_ERR("settings init: %d", err);
        return err;
    }

    err = settings_load();
    if (err) {
        LOG_ERR("settings load: %d", err);
        return err;
    }

    active_slot_ = s_active_slot;
    LOG_INF("config loaded (active recipe slot=%d, cycles=%u)", active_slot_, s_cycle_count);
    return 0;
}

int ConfigStore::loadFactory(FactoryConfig &out)
{
    out = s_factory;
    return (s_factory.version > 0) ? 0 : -ENOENT;
}

int ConfigStore::saveFactory(const FactoryConfig &cfg)
{
    s_factory = cfg;
    return settings_save_one("ptmc/factory", &s_factory, sizeof(s_factory));
}

int ConfigStore::loadRecipe(int slot, Recipe &out)
{
    if (slot < 0 || slot >= MAX_RECIPES) return -EINVAL;
    out = s_recipes[slot];
    return (s_recipes[slot].version > 0) ? 0 : -ENOENT;
}

int ConfigStore::saveRecipe(int slot, const Recipe &recipe)
{
    if (slot < 0 || slot >= MAX_RECIPES) return -EINVAL;
    s_recipes[slot] = recipe;
    char key[16];
    snprintf(key, sizeof(key), "ptmc/r/%d", slot);
    return settings_save_one(key, &s_recipes[slot], sizeof(Recipe));
}

int ConfigStore::setActiveRecipe(int slot)
{
    if (slot < 0 || slot >= MAX_RECIPES) return -EINVAL;
    active_slot_ = slot;
    s_active_slot = slot;
    return settings_save_one("ptmc/active", &s_active_slot, sizeof(s_active_slot));
}

int ConfigStore::savePosition(int axis, int32_t steps)
{
    if (axis < 0 || axis >= 3) return -EINVAL;
    s_positions[axis] = steps;
    char key[16];
    snprintf(key, sizeof(key), "ptmc/pos/%d", axis);
    return settings_save_one(key, &s_positions[axis], sizeof(int32_t));
}

int ConfigStore::loadPosition(int axis, int32_t &out)
{
    if (axis < 0 || axis >= 3) return -EINVAL;
    out = s_positions[axis];
    return 0;
}

int ConfigStore::saveCycleCount(uint32_t count)
{
    s_cycle_count = count;
    return settings_save_one("ptmc/cycles", &s_cycle_count, sizeof(s_cycle_count));
}

int ConfigStore::loadCycleCount(uint32_t &out)
{
    out = s_cycle_count;
    return 0;
}

} // namespace config
