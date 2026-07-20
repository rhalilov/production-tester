#pragma once

#include "engine/engine.h"
#include "engine/context.h"

#include <zephyr/kernel.h>

namespace app {

void init(engine::Engine *eng, engine::Context *ctx, struct k_timer *timer);

engine::Engine *engine();
engine::Context *context();

void startScan();
void stopScan();

} // namespace app
