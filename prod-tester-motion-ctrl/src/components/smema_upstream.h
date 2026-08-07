#pragma once

#include <zephyr/drivers/gpio.h>

namespace component {

struct SmemaUpstreamConfig {
    const gpio_dt_spec *ba_in;    // Board Available from upstream machine (input)
    const gpio_dt_spec *mr_out;   // Machine Ready to upstream machine (output)
};

class SmemaUpstream {
public:
    SmemaUpstream() : cfg_(nullptr) {}

    int init(const SmemaUpstreamConfig &cfg);

    bool boardAvailable() const;
    void setMachineReady(bool ready);

    const SmemaUpstreamConfig &config() const { return *cfg_; }

private:
    const SmemaUpstreamConfig *cfg_;
};

} // namespace component
