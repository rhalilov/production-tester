#include "app.h"
#include "engine/engine.h"
#include "engine/step.h"
#include "components/stepper_motor.h"
#include "components/cylinder.h"
#include "components/smema_port.h"
#include "components/sensor.h"
#include "config/config_store.h"

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

extern engine::EngineConfig machine_engine_config;
extern engine::Context machine_context;
extern int machine_init(void);
extern void config_apply_saved(void);

static config::ConfigStore config_store;
static engine::Engine sfc_engine;

static void scan_work_handler(struct k_work *work);
K_WORK_DEFINE(scan_work, scan_work_handler);

static void scan_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(scan_timer, scan_timer_handler, NULL);

static void scan_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_work_submit(&scan_work);
}

static void scan_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
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

    // Load and apply saved motor parameters from flash
    err = settings_subsys_init();
    if (err) {
        LOG_WRN("settings init: %d", err);
    } else {
        err = settings_load();
        if (err) {
            LOG_WRN("settings load: %d", err);
        } else {
            config_apply_saved();
        }
    }

    // Move cylinders to idle positions: stopper=B, rfid=B, locker=A
    LOG_INF("Cylinders to idle positions...");
    if (machine_context.stopper) machine_context.stopper->goTo(component::CylPosition::POS_B);
    if (machine_context.rfid) machine_context.rfid->goTo(component::CylPosition::POS_B);
    if (machine_context.locker) machine_context.locker->goTo(component::CylPosition::POS_A);

    // Auto-home table (after settings are applied)
    if (machine_context.table && machine_context.table->config().auto_home) {
        LOG_INF("Auto-homing table with home_rpm=%u...",
                machine_context.table->config().home_rpm);
        machine_context.table->setEnabled(true);
        err = machine_context.table->home();
        if (err) {
            LOG_ERR("table home failed: %d", err);
        }
    }

    LOG_INF("System ready. Type 'mc start' to begin.");
    return 0;
}
