#include "cylinder.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cylinder, LOG_LEVEL_INF);

#define POLL_MS 5

namespace component {

int Cylinder::init(const CylinderConfig &cfg)
{
    cfg_ = &cfg;
    interrupted_ = false;

    const gpio_dt_spec *pins[] = { cfg_->coil_a, cfg_->coil_b,
                                   cfg_->sensor_a, cfg_->sensor_b };
    const char *names[] = { "coil_a", "coil_b", "sensor_a", "sensor_b" };

    for (int i = 0; i < 4; i++) {
        if (!pins[i] || !gpio_is_ready_dt(pins[i])) {
            LOG_ERR("%s not ready", names[i]);
            return -ENODEV;
        }
    }

    gpio_pin_configure_dt(cfg_->coil_a, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(cfg_->coil_b, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(cfg_->sensor_a, GPIO_INPUT);
    gpio_pin_configure_dt(cfg_->sensor_b, GPIO_INPUT);

    LOG_INF("%s: init OK (timeout=%ums)",
            cfg_->name ? cfg_->name : "?", cfg_->timeout_ms);
    return 0;
}

bool Cylinder::sensorA() const
{
    return gpio_pin_get_dt(cfg_->sensor_a) != 0;
}

bool Cylinder::sensorB() const
{
    return gpio_pin_get_dt(cfg_->sensor_b) != 0;
}

CylPosition Cylinder::currentPos() const
{
    if (!cfg_) return CylPosition::UNKNOWN;
    bool a = sensorA();
    bool b = sensorB();
    if (a && !b) return CylPosition::POS_A;
    if (b && !a) return CylPosition::POS_B;
    return CylPosition::UNKNOWN;
}

Cylinder::Error Cylinder::goTo(CylPosition pos)
{
    if (!cfg_) return Error::NOT_READY;

    interrupted_ = false;

    const gpio_dt_spec *coil_on;
    const gpio_dt_spec *coil_off;
    bool (*check_sensor)(const Cylinder *);

    if (pos == CylPosition::POS_A) {
        coil_on = cfg_->coil_a;
        coil_off = cfg_->coil_b;
        check_sensor = [](const Cylinder *c) { return c->sensorA(); };
    } else if (pos == CylPosition::POS_B) {
        coil_on = cfg_->coil_b;
        coil_off = cfg_->coil_a;
        check_sensor = [](const Cylinder *c) { return c->sensorB(); };
    } else {
        return Error::NOT_READY;
    }

    // Already there?
    if (check_sensor(this)) {
        return Error::OK;
    }

    // De-energize opposite, energize target
    gpio_pin_set_dt(coil_off, 0);
    gpio_pin_set_dt(coil_on, 1);

    // Wait for confirm sensor (LEVEL-until-confirm)
    uint32_t waited = 0;
    while (!check_sensor(this)) {
        if (interrupted_) {
            off();
            return Error::INTERRUPTED;
        }
        if (cfg_->timeout_ms > 0 && waited >= cfg_->timeout_ms) {
            off();
            LOG_ERR("%s: confirm timeout (%ums)",
                    cfg_->name ? cfg_->name : "?", cfg_->timeout_ms);
            return Error::TIMEOUT;
        }
        k_msleep(POLL_MS);
        waited += POLL_MS;
    }

    // Confirmed — coils off (bistable valve stays in position)
    off();
    return Error::OK;
}

void Cylinder::off()
{
    if (!cfg_) return;
    gpio_pin_set_dt(cfg_->coil_a, 0);
    gpio_pin_set_dt(cfg_->coil_b, 0);
}

void Cylinder::setTimeout(uint32_t ms)
{
    if (cfg_) {
        const_cast<CylinderConfig *>(cfg_)->timeout_ms = ms;
    }
}

} // namespace component
