#pragma once

#include "recipe.h"

#include <stdint.h>

namespace config {

static constexpr int MAX_RECIPES = 8;

class ConfigStore {
public:
    int init();

    // Factory config (Level 1)
    int loadFactory(FactoryConfig &out);
    int saveFactory(const FactoryConfig &cfg);

    // Recipes (Level 2)
    int loadRecipe(int slot, Recipe &out);
    int saveRecipe(int slot, const Recipe &recipe);
    int activeRecipeSlot() const { return active_slot_; }
    int setActiveRecipe(int slot);

    // Runtime state
    int savePosition(int axis, int32_t steps);
    int loadPosition(int axis, int32_t &out);
    int saveCycleCount(uint32_t count);
    int loadCycleCount(uint32_t &out);

private:
    int active_slot_;
};

} // namespace config
