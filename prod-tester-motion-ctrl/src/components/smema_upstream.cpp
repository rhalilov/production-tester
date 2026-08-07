#include "smema_upstream.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(smema_up, LOG_LEVEL_INF);

namespace component {

int SmemaUpstream::init(const SmemaUpstreamConfig &cfg)
{
    cfg_ = &cfg;

    if (cfg_->ba_in && gpio_is_ready_dt(cfg_->ba_in)) {
        int err = gpio_pin_configure_dt(cfg_->ba_in, GPIO_INPUT);
        if (err) { LOG_ERR("ba_in config: %d", err); return err; }
    }

    if (cfg_->mr_out && gpio_is_ready_dt(cfg_->mr_out)) {
        int err = gpio_pin_configure_dt(cfg_->mr_out, GPIO_OUTPUT_INACTIVE);
        if (err) { LOG_ERR("mr_out config: %d", err); return err; }
    }

    LOG_INF("SMEMA upstream init OK");
    return 0;
}

bool SmemaUpstream::boardAvailable() const
{
    if (!cfg_ || !cfg_->ba_in) return false;
    return gpio_pin_get_dt(cfg_->ba_in) != 0;
}

void SmemaUpstream::setMachineReady(bool ready)
{
    if (cfg_ && cfg_->mr_out) {
        gpio_pin_set_dt(cfg_->mr_out, ready ? 1 : 0);
    }
}

} // namespace component
