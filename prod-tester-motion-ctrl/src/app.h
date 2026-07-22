#pragma once

#include "engine/engine.h"
#include "engine/context.h"

#include <zephyr/kernel.h>

namespace app {

enum class AccessLevel : uint8_t {
    OPERATOR,   // can only start/stop, select recipe
    PROC_ENG,   // process engineer: speeds, limits, positions
    FACTORY,    // full access, soft limits disabled
};

void init(engine::Engine *eng, engine::Context *ctx, struct k_timer *timer);

engine::Engine *engine();
engine::Context *context();

AccessLevel accessLevel();
void setAccessLevel(AccessLevel level);
bool isFactory();

void startScan();
void stopScan();

} // namespace app
