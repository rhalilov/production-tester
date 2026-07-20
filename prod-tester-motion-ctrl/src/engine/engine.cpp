#include "engine.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(engine, LOG_LEVEL_INF);

namespace engine {

int Engine::init(const EngineConfig &cfg, Context &ctx)
{
    cfg_ = &cfg;
    current_step_ = 0;
    advance_requested_ = false;
    mode_ = OperatingMode::AUTO;
    running_ = false;
    cycle_count_ = 0;

    LOG_INF("SFC engine init: %d steps, scan=%ums", cfg_->step_count, cfg_->scan_period_ms);
    return 0;
}

void Engine::start()
{
    if (!running_ && cfg_ && cfg_->step_count > 0) {
        running_ = true;
        current_step_ = 0;
        LOG_INF("Engine started (mode=%d)", (int)mode_);
    }
}

void Engine::stop()
{
    running_ = false;
    LOG_INF("Engine stopped at step %d", current_step_);
}

const StepDef *Engine::currentStep() const
{
    if (!cfg_ || current_step_ < 0 || current_step_ >= cfg_->step_count) {
        return nullptr;
    }
    return &cfg_->steps[current_step_];
}

void Engine::enterStep(Context &ctx, int index)
{
    if (index < 0 || index >= cfg_->step_count) {
        // Sequence complete — loop back to start
        index = 0;
        cycle_count_++;
        LOG_INF("Cycle #%u complete", cycle_count_);
    }

    const StepDef *prev = currentStep();
    if (prev && prev->on_exit) {
        prev->on_exit(ctx);
    }

    current_step_ = index;
    const StepDef *step = &cfg_->steps[current_step_];

    LOG_INF("-> step[%d] \"%s\"", current_step_, step->name ? step->name : "?");

    if (step->on_entry) {
        step->on_entry(ctx);
    }
}

int Engine::resolveNext()
{
    const StepDef *step = currentStep();
    if (!step) return 0;

    // Check branches first
    if (step->branches && step->branch_count > 0) {
        // Branches are evaluated in scan() context — we don't have ctx here,
        // so branch evaluation is done inline in scan()
    }

    int next = step->default_next;
    if (next < 0) {
        return 0;   // end of sequence -> loop
    }
    return next;
}

void Engine::scan(Context &ctx)
{
    if (!running_ || !cfg_) {
        return;
    }

    // 1. Safety interlocks — every scan, before logic
    if (cfg_->interlock_check) {
        cfg_->interlock_check(ctx, faults_);
    }

    if (faults_.inFault()) {
        return; // Frozen in fault — no sequence progress
    }

    const StepDef *step = currentStep();
    if (!step) {
        return;
    }

    // 2. Active action (runs every scan while step is active)
    if (step->on_active) {
        step->on_active(ctx);
    }

    // 3. In STEP mode, wait for explicit advance
    if (mode_ == OperatingMode::STEP && !advance_requested_) {
        return;
    }

    // 4. Check transition condition
    if (step->transition && step->transition(ctx)) {
        advance_requested_ = false;

        // Check branches for conditional next
        int next = -1;
        if (step->branches) {
            for (int i = 0; i < step->branch_count; i++) {
                if (step->branches[i].condition && step->branches[i].condition(ctx)) {
                    next = step->branches[i].target_step;
                    break;
                }
            }
        }
        if (next < 0) {
            next = step->default_next;
            if (next < 0) {
                next = current_step_ + 1;
            }
        }

        enterStep(ctx, next);
    }
}

void Engine::gotoStep(int index)
{
    if (cfg_ && index >= 0 && index < cfg_->step_count) {
        // Direct jump (use with care — only in MANUAL/recovery)
        current_step_ = index;
        LOG_INF("goto step[%d] \"%s\"", index,
                cfg_->steps[index].name ? cfg_->steps[index].name : "?");
    }
}

} // namespace engine
