#pragma once

#include <stdint.h>

namespace engine {

struct Context;

struct Branch {
    bool (*condition)(Context &ctx);
    int target_step;
};

struct StepDef {
    const char *name;
    void (*on_entry)(Context &ctx);
    void (*on_active)(Context &ctx);    // nullable: run every scan while active
    void (*on_exit)(Context &ctx);      // nullable: run once on exit
    bool (*transition)(Context &ctx);   // condition to advance to next
    int default_next;                   // -1 = end of sequence (loop back to 0)
    Branch *branches;                   // nullable: conditional jumps
    int branch_count;
};

struct RetryPolicy {
    int checkpoint_step;                // step index to return to
    const StepDef *recovery_seq;        // recovery steps (nullable = direct jump)
    int recovery_seq_len;
};

} // namespace engine
