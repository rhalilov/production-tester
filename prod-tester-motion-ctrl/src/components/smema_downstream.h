#pragma once

#include <zephyr/drivers/gpio.h>

namespace component {

struct SmemaDownstreamConfig {
    const gpio_dt_spec *mr_in;          // Machine Ready from downstream machine (input)
    const gpio_dt_spec *ba_out;         // Board Available to downstream machine (output)
    const gpio_dt_spec *ba_fail_out;    // Board Available NG to downstream (output, nullable)
};

class SmemaDownstream {
public:
    SmemaDownstream() : cfg_(nullptr) {}

    int init(const SmemaDownstreamConfig &cfg);

    bool machineReady() const;
    void setBoardAvailable(bool available);
    void setBoardAvailableFail(bool available);
    void allOff();

    const SmemaDownstreamConfig &config() const { return *cfg_; }

private:
    const SmemaDownstreamConfig *cfg_;
};

} // namespace component
