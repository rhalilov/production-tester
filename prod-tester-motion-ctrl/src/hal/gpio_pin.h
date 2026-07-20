#pragma once

#include <zephyr/drivers/gpio.h>

namespace hal {

class GpioPin {
public:
    GpioPin() : spec_{} {}
    explicit GpioPin(const gpio_dt_spec &spec) : spec_(spec) {}

    int initOutput(bool initial_value = false);
    int initInput();

    void set(bool on);
    bool read() const;

    bool isReady() const;
    const gpio_dt_spec &spec() const { return spec_; }

private:
    gpio_dt_spec spec_;
};

class GpioInterrupt {
public:
    using Callback = void(*)(void *context);

    GpioInterrupt() : spec_{}, cb_(nullptr), ctx_(nullptr) {}
    explicit GpioInterrupt(const gpio_dt_spec &spec) : spec_(spec), cb_(nullptr), ctx_(nullptr) {}

    int init(Callback cb, void *ctx, gpio_flags_t edge = GPIO_INT_EDGE_BOTH);

private:
    gpio_dt_spec spec_;
    gpio_callback cb_data_;
    Callback cb_;
    void *ctx_;

    static void isrTrampoline(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
};

} // namespace hal
