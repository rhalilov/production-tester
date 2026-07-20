#include "app.h"
#include "engine/engine.h"
#include "engine/step.h"
#include "components/stepper_motor.h"
#include "components/cylinder.h"
#include "components/smema_port.h"
#include "components/sensor.h"
#include "config/config_store.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

extern engine::EngineConfig machine_engine_config;
extern engine::Context machine_context;
extern int machine_init(void);

static config::ConfigStore config_store;
static engine::Engine sfc_engine;

static void scan_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(scan_timer, scan_timer_handler, NULL);

static void scan_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    sfc_engine.scan(machine_context);
}

int main(void)
{
    LOG_INF("prod-tester-motion-ctrl v0.1.0 starting");

    int err = config_store.init();
    if (err) {
        LOG_WRN("config store init: %d (continuing with defaults)", err);
    }

    err = machine_init();
    if (err) {
        LOG_ERR("machine init failed: %d", err);
        return err;
    }

    err = sfc_engine.init(machine_engine_config, machine_context);
    if (err) {
        LOG_ERR("engine init failed: %d", err);
        return err;
    }

    app::init(&sfc_engine, &machine_context, &scan_timer);

    LOG_INF("System ready. Type 'mc start' to begin.");
    return 0;
}
