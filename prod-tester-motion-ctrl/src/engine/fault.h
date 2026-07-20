#pragma once

#include <stdint.h>

namespace engine {

enum class FaultState : uint8_t {
    NORMAL,
    FAULT,
    RECOVERY,
};

class FaultManager {
public:
    FaultManager() : state_(FaultState::NORMAL), reason_(nullptr) {}

    void raise(const char *reason);
    void reset();
    void enterRecovery();
    void recoveryComplete();
    void recoveryFailed();

    FaultState state() const { return state_; }
    bool inFault() const { return state_ != FaultState::NORMAL; }
    const char *reason() const { return reason_; }

private:
    FaultState state_;
    const char *reason_;
};

} // namespace engine
