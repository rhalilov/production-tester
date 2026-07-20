#include "app.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

namespace app {

static engine::Engine *s_engine;
static engine::Context *s_ctx;
static struct k_timer *s_timer;

void init(engine::Engine *eng, engine::Context *ctx, struct k_timer *timer)
{
    s_engine = eng;
    s_ctx = ctx;
    s_timer = timer;
}

engine::Engine *engine() { return s_engine; }
engine::Context *context() { return s_ctx; }

void startScan()
{
    if (s_timer) {
        k_timer_start(s_timer, K_MSEC(CONFIG_SCAN_PERIOD_MS),
                      K_MSEC(CONFIG_SCAN_PERIOD_MS));
        LOG_INF("Scan timer started (%d ms)", CONFIG_SCAN_PERIOD_MS);
    }
}

void stopScan()
{
    if (s_timer) {
        k_timer_stop(s_timer);
        LOG_INF("Scan timer stopped");
    }
}

} // namespace app
