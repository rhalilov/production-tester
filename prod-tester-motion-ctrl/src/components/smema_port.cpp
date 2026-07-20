#include "smema_port.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(smema, LOG_LEVEL_INF);

namespace component {

int SmemaPort::init(const SmemaPortConfig &cfg)
{
    cfg_ = &cfg;

    struct { const gpio_dt_spec *pin; gpio_flags_t flags; const char *name; } pins[] = {
        { cfg_->board_available_in,       GPIO_INPUT,           "ba_in" },
        { cfg_->machine_ready_out,        GPIO_OUTPUT_INACTIVE, "mr_out" },
        { cfg_->board_available_out,      GPIO_OUTPUT_INACTIVE, "ba_out" },
        { cfg_->machine_ready_in,         GPIO_INPUT,           "mr_in" },
    };

    for (auto &p : pins) {
        if (p.pin && gpio_is_ready_dt(p.pin)) {
            int err = gpio_pin_configure_dt(p.pin, p.flags);
            if (err) {
                LOG_ERR("SMEMA %s config err: %d", p.name, err);
                return err;
            }
        }
    }

    if (cfg_->board_available_fail_out && gpio_is_ready_dt(cfg_->board_available_fail_out)) {
        gpio_pin_configure_dt(cfg_->board_available_fail_out, GPIO_OUTPUT_INACTIVE);
    }

    LOG_INF("SMEMA port init OK");
    return 0;
}

bool SmemaPort::boardAvailableIn() const
{
    if (!cfg_ || !cfg_->board_available_in) return false;
    return gpio_pin_get_dt(cfg_->board_available_in) != 0;
}

void SmemaPort::setMachineReadyOut(bool ready)
{
    if (cfg_ && cfg_->machine_ready_out) {
        gpio_pin_set_dt(cfg_->machine_ready_out, ready ? 1 : 0);
    }
}

bool SmemaPort::machineReadyIn() const
{
    if (!cfg_ || !cfg_->machine_ready_in) return false;
    return gpio_pin_get_dt(cfg_->machine_ready_in) != 0;
}

void SmemaPort::setBoardAvailableOut(bool available)
{
    if (cfg_ && cfg_->board_available_out) {
        gpio_pin_set_dt(cfg_->board_available_out, available ? 1 : 0);
    }
}

void SmemaPort::setBoardAvailableFailOut(bool available)
{
    if (cfg_ && cfg_->board_available_fail_out) {
        gpio_pin_set_dt(cfg_->board_available_fail_out, available ? 1 : 0);
    }
}

void SmemaPort::allOff()
{
    setMachineReadyOut(false);
    setBoardAvailableOut(false);
    setBoardAvailableFailOut(false);
}

} // namespace component
