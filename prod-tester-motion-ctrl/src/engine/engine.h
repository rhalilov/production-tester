#pragma once

#include "step.h"
#include "fault.h"
#include "timer.h"

#include <stdint.h>

namespace engine {

enum class OperatingMode : uint8_t {
    AUTO,
    STEP,
    MANUAL,
};

struct Context;

using InterlockCheck = void(*)(Context &ctx, FaultManager &faults);

struct EngineConfig {
    const StepDef *steps;
    int step_count;
    InterlockCheck interlock_check;     // called every scan
    uint32_t scan_period_ms;
};

class Engine {
public:
    Engine() : cfg_(nullptr), current_step_(0), advance_requested_(false),
               mode_(OperatingMode::AUTO), running_(false), cycle_count_(0) {}

    int init(const EngineConfig &cfg, Context &ctx);
    void start();
    void stop();

    // Single scan cycle — call from a periodic timer/thread
    void scan(Context &ctx);

    // Mode control
    void setMode(OperatingMode mode) { mode_ = mode; }
    OperatingMode mode() const { return mode_; }

    // Step control (STEP mode)
    void advanceStep() { advance_requested_ = true; }
    void gotoStep(int index);

    // State
    int currentStepIndex() const { return current_step_; }
    const StepDef *currentStep() const;
    bool isRunning() const { return running_; }
    uint32_t cycleCount() const { return cycle_count_; }
    FaultManager &faults() { return faults_; }

    const EngineConfig &config() const { return *cfg_; }

private:
    const EngineConfig *cfg_;
    int current_step_;
    volatile bool advance_requested_;
    OperatingMode mode_;
    bool running_;
    uint32_t cycle_count_;
    FaultManager faults_;

    void enterStep(Context &ctx, int index);
    int resolveNext();
};

} // namespace engine
