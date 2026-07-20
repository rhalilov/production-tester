/*
 * safety — owns the single supervisor thread, the fault state machine, the
 * motor ENABLE lines and the IWDG. Wakes on an io interrupt event (or a short
 * poll tick), refreshes io, latches faults on ALM, and stops motion. Policy
 * lives here; pins live in io.
 */
#ifndef INFEED_SAFETY_H_
#define INFEED_SAFETY_H_

#include <stdbool.h>

#define SAFETY_AXES 3

int  safety_init(void);     /* configures ENA + IWDG, starts the supervisor */
bool safety_in_fault(void);
void safety_fault_reset(void);            /* clear the latch (leaves motors free) */
void safety_raise_fault(const char *why); /* latch a fault from app code (e.g. a timeout) */

/* Per-bank ENABLE (axis 0..SAFETY_AXES-1). set returns -EINVAL on a bad axis,
 * -EBUSY if a fault is latched and on=true. */
int  safety_set_enable(int axis, bool on);
bool safety_enabled(int axis);

#endif /* INFEED_SAFETY_H_ */
