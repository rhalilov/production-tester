#include "smema_downstream.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(smema_dn, LOG_LEVEL_INF);

namespace component {

int SmemaDownstream::init(const SmemaDownstreamConfig &cfg)
{
    cfg_ = &cfg;

    if (cfg_->mr_in && gpio_is_ready_dt(cfg_->mr_in)) {
        int err = gpio_pin_configure_dt(cfg_->mr_in, GPIO_INPUT);
        if (err) { LOG_ERR("mr_in config: %d", err); return err; }
    }

    if (cfg_->ba_out && gpio_is_ready_dt(cfg_->ba_out)) {
        int err = gpio_pin_configure_dt(cfg_->ba_out, GPIO_OUTPUT_INACTIVE);
        if (err) { LOG_ERR("ba_out config: %d", err); return err; }
    }

    if (cfg_->ba_fail_out && gpio_is_ready_dt(cfg_->ba_fail_out)) {
        int err = gpio_pin_configure_dt(cfg_->ba_fail_out, GPIO_OUTPUT_INACTIVE);
        if (err) { LOG_ERR("ba_fail_out config: %d", err); return err; }
    }

    LOG_INF("SMEMA downstream init OK");
    return 0;
}

bool SmemaDownstream::machineReady() const
{
    if (!cfg_ || !cfg_->mr_in) return false;
    return gpio_pin_get_dt(cfg_->mr_in) != 0;
}

void SmemaDownstream::setBoardAvailable(bool available)
{
    if (cfg_ && cfg_->ba_out) {
        gpio_pin_set_dt(cfg_->ba_out, available ? 1 : 0);
    }
}

void SmemaDownstream::setBoardAvailableFail(bool available)
{
    if (cfg_ && cfg_->ba_fail_out) {
        gpio_pin_set_dt(cfg_->ba_fail_out, available ? 1 : 0);
    }
}

void SmemaDownstream::allOff()
{
    setBoardAvailable(false);
    setBoardAvailableFail(false);
}

} // namespace component
