#pragma once

#include <zephyr/drivers/gpio.h>

namespace component {

struct SmemaPortConfig {
    const gpio_dt_spec *board_available_in;    // input: upstream/downstream
    const gpio_dt_spec *machine_ready_out;     // output: we are ready / accepting
    const gpio_dt_spec *board_available_out;   // output: we have a board
    const gpio_dt_spec *machine_ready_in;      // input: they are ready
    const gpio_dt_spec *board_available_fail_out; // output: NG board available (nullable)
};

class SmemaPort {
public:
    SmemaPort() : cfg_(nullptr) {}

    int init(const SmemaPortConfig &cfg);

    // Upstream side (we are the receiver)
    bool boardAvailableIn() const;
    void setMachineReadyOut(bool ready);

    // Downstream side (we are the sender)
    bool machineReadyIn() const;
    void setBoardAvailableOut(bool available);
    void setBoardAvailableFailOut(bool available);

    void allOff();

private:
    const SmemaPortConfig *cfg_;
};

} // namespace component
