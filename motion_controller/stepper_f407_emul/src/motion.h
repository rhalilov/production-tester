/*
 * motion — the 3 step/dir axes. Bank 3 (axis3) is one logical axis driving the
 * two synchronous motors. Two primitives the sequence is built from:
 *   - motion_go()  : blocking, fixed-distance move then stop.
 *   - motion_run() : start a continuous run; caller stops it (e.g. on a sensor).
 * Also owns the on-board motion LEDs.
 */
#ifndef INFEED_MOTION_H_
#define INFEED_MOTION_H_

#include <stdbool.h>
#include <stdint.h>

enum motion_axis { MOTION_AXIS1, MOTION_AXIS2, MOTION_AXIS3, MOTION_AXIS_COUNT };
enum motion_dir  { MOTION_NEG = -1, MOTION_POS = 1 };

#define MOTION_STEPS_PER_REV  2000U   /* drive set to 2000 ppr (P001=10) */

int  motion_init(void);

/* Blocking: move `revs` revolutions at `rpm` in `dir`, then stop. */
int  motion_go(enum motion_axis axis, enum motion_dir dir, uint32_t revs, uint32_t rpm);
/* Blocking: same, but in raw steps (for sub-revolution moves, e.g. the bob). */
int  motion_go_steps(enum motion_axis axis, enum motion_dir dir, uint32_t steps, uint32_t rpm);

/* Per-axis travel-sense invert (runtime; persisted by the roles layer). When
 * set, a logical POS command spins the motor the other way physically — the
 * commissioning fix for a reversed drive. cur_dir/LEDs stay on the logical dir. */
void motion_set_invert(enum motion_axis axis, bool invert);
bool motion_invert(enum motion_axis axis);
/* Non-blocking: run continuously at `rpm` in `dir` until motion_stop(). */
int  motion_run(enum motion_axis axis, enum motion_dir dir, uint32_t rpm);
void motion_stop(enum motion_axis axis);
void motion_stop_all(void);

bool motion_is_moving(enum motion_axis axis);
int  motion_dir(enum motion_axis axis);    /* +1 / -1 / 0 (stopped) */

/* Update the on-board indicators: LED3=motor1 moving, LED4/LED5=table +/-,
 * LED4+LED5=table at home. Caller supplies the at-home condition. */
void motion_leds_update(bool axis3_at_home);

#endif /* INFEED_MOTION_H_ */
