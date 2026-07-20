#include "fault.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fault, LOG_LEVEL_INF);

namespace engine {

void FaultManager::raise(const char *reason)
{
    if (state_ == FaultState::NORMAL || state_ == FaultState::RECOVERY) {
        state_ = FaultState::FAULT;
        reason_ = reason;
        LOG_ERR("FAULT: %s", reason ? reason : "unknown");
    }
}

void FaultManager::reset()
{
    if (state_ == FaultState::FAULT) {
        state_ = FaultState::NORMAL;
        reason_ = nullptr;
        LOG_INF("Fault cleared");
    }
}

void FaultManager::enterRecovery()
{
    if (state_ == FaultState::FAULT) {
        state_ = FaultState::RECOVERY;
        LOG_INF("Entering recovery");
    }
}

void FaultManager::recoveryComplete()
{
    if (state_ == FaultState::RECOVERY) {
        state_ = FaultState::NORMAL;
        reason_ = nullptr;
        LOG_INF("Recovery complete");
    }
}

void FaultManager::recoveryFailed()
{
    if (state_ == FaultState::RECOVERY) {
        state_ = FaultState::FAULT;
        LOG_ERR("Recovery failed — staying in FAULT");
    }
}

} // namespace engine
