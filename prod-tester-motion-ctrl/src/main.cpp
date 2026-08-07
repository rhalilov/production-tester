#include "app.h"
#include "engine/engine.h"
#include "engine/step.h"
#include "components/stepper_motor.h"
#include "components/cylinder.h"
#include "components/smema_port.h"
#include "components/sensor.h"
#include "config/config_store.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

extern engine::EngineConfig machine_engine_config;
extern engine::Context machine_context;
extern int machine_init(void);
extern void config_apply_saved(void);
extern void config_try_blob_fallback(void);
extern uint8_t config_cyl_idle_pos(int idx);

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
    /* Keep USB OTG power switch U6 off (PC0 HIGH = VBUS disabled) */
    static const struct gpio_dt_spec usb_pwr =
        GPIO_DT_SPEC_GET(DT_NODELABEL(usb_pwr), gpios);
    gpio_pin_configure_dt(&usb_pwr, GPIO_OUTPUT_INACTIVE);

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

    config_try_blob_fallback();

    // Move cylinders to idle positions — reverse of arm order (clear path first)
    // Order: rfid, locker, stopper (mirrors release sequence: steps 12→13→14)
    LOG_INF("Cylinders to idle positions...");
    component::Cylinder *cyls[] = { machine_context.stopper, machine_context.rfid, machine_context.locker };
    int reset_order[] = { 1, 2, 0 };  // rfid, locker, stopper
    for (int i = 0; i < 3; i++) {
        int idx = reset_order[i];
        if (cyls[idx]) {
            auto pos = config_cyl_idle_pos(idx) ? component::CylPosition::POS_B : component::CylPosition::POS_A;
            cyls[idx]->goTo(pos);
        }
    }

    // Auto-home head (after settings are applied)
    auto *head_motor = machine_context.head->motor();
    if (head_motor && head_motor->config().auto_home) {
        LOG_INF("Auto-homing head with home_rpm=%u...",
                head_motor->config().home_rpm);
        head_motor->setEnabled(true);
        err = machine_context.head->home();
        if (err) {
            LOG_ERR("head home failed: %d", err);
        }
    }

    LOG_INF("Auto-starting sequence...");
    machine_context.conveyor->beltMotor()->setEnabled(true);
    machine_context.head->motor()->setEnabled(true);
    sfc_engine.setMode(engine::OperatingMode::AUTO);
    sfc_engine.start();
    app::startScan();
    LOG_INF("System running.");
    return 0;
}
