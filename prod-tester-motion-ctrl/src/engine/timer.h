#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>

namespace engine {

class SoftTimer {
public:
    SoftTimer() : target_(0), running_(false) {}

    void start(uint32_t ms) {
        target_ = k_uptime_get_32() + ms;
        running_ = true;
    }

    bool expired() const {
        if (!running_) return false;
        return k_uptime_get_32() >= target_;
    }

    void stop() { running_ = false; }
    bool isRunning() const { return running_; }

    uint32_t elapsed() const {
        if (!running_) return 0;
        uint32_t now = k_uptime_get_32();
        uint32_t start = target_ - (target_ > now ? target_ - now : 0);
        return now >= target_ ? target_ : now - (target_ - (target_ - now));
    }

private:
    uint32_t target_;
    bool running_;
};

} // namespace engine
