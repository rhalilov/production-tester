#include "gpio_pin.h"

namespace hal {

int GpioPin::initOutput(bool initial_value)
{
    if (!isReady()) {
        return -ENODEV;
    }
    return gpio_pin_configure_dt(&spec_,
        initial_value ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE);
}

int GpioPin::initInput()
{
    if (!isReady()) {
        return -ENODEV;
    }
    return gpio_pin_configure_dt(&spec_, GPIO_INPUT);
}

void GpioPin::set(bool on)
{
    gpio_pin_set_dt(&spec_, on ? 1 : 0);
}

bool GpioPin::read() const
{
    return gpio_pin_get_dt(&spec_) != 0;
}

bool GpioPin::isReady() const
{
    return gpio_is_ready_dt(&spec_);
}

// --- GpioInterrupt ---

void GpioInterrupt::isrTrampoline(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    GpioInterrupt *self = CONTAINER_OF(cb, GpioInterrupt, cb_data_);
    if (self->cb_) {
        self->cb_(self->ctx_);
    }
}

int GpioInterrupt::init(Callback cb, void *ctx, gpio_flags_t edge)
{
    if (!gpio_is_ready_dt(&spec_)) {
        return -ENODEV;
    }

    cb_ = cb;
    ctx_ = ctx;

    int err = gpio_pin_configure_dt(&spec_, GPIO_INPUT);
    if (err) {
        return err;
    }

    err = gpio_pin_interrupt_configure_dt(&spec_, edge);
    if (err) {
        return err;
    }

    gpio_init_callback(&cb_data_, isrTrampoline, BIT(spec_.pin));
    return gpio_add_callback(spec_.port, &cb_data_);
}

} // namespace hal
