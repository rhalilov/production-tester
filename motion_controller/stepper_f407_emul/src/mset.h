/*
 * mset — operator-settable motor speeds/rounds (`infeed set 1..7`).
 * Persisted to flash via `infeed cfg save` (same ZMS settings backend as the
 * sensor map). Rounds are stored as raw steps (rounds * MOTION_STEPS_PER_REV).
 * Defaults = the current firmware values until an operator changes them.
 */
#ifndef INFEED_MSET_H_
#define INFEED_MSET_H_

#include <stdbool.h>
#include <stdint.h>

enum mset_id {
	MSET_HOME = 1,   /* set 1: table home/up rpm       (#1 + #18) */
	MSET_CONVEY,     /* set 2: conveyor rpm -> Laser 2  (#5)  */
	MSET_CREEP,      /* set 3: conveyor rpm + rounds    (#9)  */
	MSET_DOWN1,      /* set 4: table down phase 1 rpm+r (#10) */
	MSET_DOWN2,      /* set 5: table down phase 2 rpm+r (#10) */
	MSET_OUT,        /* set 6: conveyor rpm -> Laser 3  (#25) */
	MSET_EJECT,      /* set 7: conveyor eject rpm       (#30) */
	MSET_COUNT = MSET_EJECT,
};

uint32_t    mset_rpm(enum mset_id id);       /* configured rpm */
uint32_t    mset_steps(enum mset_id id);     /* rounds * 2000; 0 if rpm-only */
bool        mset_has_rounds(enum mset_id id);/* set 3/4/5 take rounds too */
const char *mset_name(enum mset_id id);      /* short label for help/status */

/* Set rpm (always) and steps (only used when the id has rounds). Returns
 * -EINVAL on a bad id or rpm==0. RAM only — persist with mset_save(). */
int  mset_set(enum mset_id id, uint32_t rpm, uint32_t steps);

int  mset_save(void);   /* write all 7 to flash (called by `infeed cfg save`) */

#endif /* INFEED_MSET_H_ */
