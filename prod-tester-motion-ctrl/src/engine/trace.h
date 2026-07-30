#pragma once

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define TRACE(tag, fmt, ...) \
    printk("[%07u] " tag ": " fmt "\n", (unsigned)k_uptime_get(), ##__VA_ARGS__)

#define TRACE_STEP(name)       TRACE("STEP", "%s", name)
#define TRACE_ACT(fmt, ...)    TRACE("ACT", fmt, ##__VA_ARGS__)
#define TRACE_WAIT(fmt, ...)   TRACE("WAIT", fmt, ##__VA_ARGS__)
#define TRACE_DONE(fmt, ...)   TRACE("DONE", fmt, ##__VA_ARGS__)
#define TRACE_ERR(fmt, ...)    TRACE("ERR", fmt, ##__VA_ARGS__)
