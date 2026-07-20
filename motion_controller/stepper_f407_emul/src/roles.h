/*
 * roles — assignable sensor function map. The process references logical roles
 * ("panel_near"), not physical sensors. Each role is bound to one of the 11
 * sensors; the binding + each sensor's active level are CLI-settable and
 * persisted to flash (NVS), so the same firmware adapts to different fixtures
 * without recompiling.
 */
#ifndef INFEED_ROLES_H_
#define INFEED_ROLES_H_

#include <stdbool.h>
#include "io.h"

enum io_role {
	ROLE_PANEL_PRESENTED,    /* #3  infeed trigger          */
	ROLE_PANEL_NEAR,         /* #6  near position -> creep  */
	ROLE_PANEL_IN_POSITION,  /* #17 at outfeed              */
	ROLE_TABLE_HOME,         /* homing + #12 (Motors 3/4)   */
	ROLE_CONVEYOR_WIDTH,     /* reserved (automation TBD)   */
	ROLE_CYL1_DOWN, ROLE_CYL1_UP,   /* reserved (open-loop) */
	ROLE_CYL2_DOWN, ROLE_CYL2_UP,
	ROLE_CYL3_DOWN, ROLE_CYL3_UP,
	ROLE_COUNT
};

/* Bring up the map: load compiled defaults, then any flash-persisted overrides. */
int roles_init(void);

/* Asserted state of the sensor currently bound to a role. */
bool role_read(enum io_role r);
enum io_sensor_id role_sensor(enum io_role r);

/* Bind a sensor to a role (RAM only — call roles_save() to persist). */
int role_assign(enum io_role r, enum io_sensor_id s);

const char *role_name(enum io_role r);
int role_by_name(const char *name);     /* -1 if unknown */
int sensor_by_name(const char *name);   /* -1 if unknown */

int roles_save(void);    /* persist map + polarity + direction to flash */
int roles_reset(void);   /* restore compiled defaults, then save */

/* true once a saved config has been loaded from flash (gates the safe-hold). */
bool roles_is_configured(void);

/* Portable config token "INF1:<hex>" — dump current map+polarity+direction,
 * or apply a token to RAM (validate; caller persists with roles_save). */
int roles_dump(char *out, size_t n);
int roles_apply_token(const char *tok);

#endif /* INFEED_ROLES_H_ */
