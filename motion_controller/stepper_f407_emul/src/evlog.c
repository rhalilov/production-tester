#include "evlog.h"

#include <stdarg.h>
#include <stdio.h>
#include <zephyr/kernel.h>

/*
 * Per-step timestamps are still computed (the cycle clock keeps running) but are
 * MASKED in the printed output for now. Set EVLOG_SHOW_TIME to 1 to bring back
 * the leading [SSSS.S] cycle-relative stamp on every line + the cycle total.
 */
#define EVLOG_SHOW_TIME 0

static int64_t cycle_start;

#if EVLOG_SHOW_TIME
static void stamp(char *buf, size_t n)
{
	uint32_t ms = (uint32_t)(k_uptime_get() - cycle_start);

	(void)snprintf(buf, n, "%4u.%u", ms / 1000U, (ms % 1000U) / 100U);
}
#endif

void evlog_cycle_start(unsigned cycle)
{
	cycle_start = k_uptime_get();   /* reset the clock even while masked */
	printk("==== Cycle %u ====\n", cycle);
}

void evlog_cycle_end(unsigned cycle)
{
#if EVLOG_SHOW_TIME
	char t[12];

	stamp(t, sizeof(t));
	printk("==== Cycle %u done: %s s ====\n", cycle, t);
#else
	printk("==== Cycle %u done ====\n", cycle);
#endif
}

void evlog(int step, char dir, const char *fmt, ...)
{
	char msg[160];
	va_list ap;

	va_start(ap, fmt);
	(void)vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);

#if EVLOG_SHOW_TIME
	char t[12];

	stamp(t, sizeof(t));
	printk("[%s] #%-2d %c %s\n", t, step, dir, msg);
#else
	printk("#%-2d %c %s\n", step, dir, msg);
#endif
}
