#include "mset.h"
#include "motion.h"

#include <errno.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mset, LOG_LEVEL_INF);

struct row {
	uint32_t rpm;
	uint32_t steps;   /* rounds * MOTION_STEPS_PER_REV; 0 for rpm-only rows */
	bool     rounds;  /* does this setting take a rounds value? */
	const char *name;
};

/* Defaults = current firmware values (indexed by enum mset_id, 1..7). */
static struct row rows[MSET_COUNT + 1] = {
	[MSET_HOME]   = { 10, 0,                        false, "table home/up rpm (#1,#18)" },
	[MSET_CONVEY] = { 40, 0,                        false, "convey->Laser2 rpm (#5)" },
	[MSET_CREEP]  = { 10, MOTION_STEPS_PER_REV / 2, true,  "creep rpm+rev (#9)" },
	[MSET_DOWN1]  = { 40, 5 * MOTION_STEPS_PER_REV, true,  "table down p1 rpm+rev (#10)" },
	[MSET_DOWN2]  = { 10, 1 * MOTION_STEPS_PER_REV, true,  "table down p2 rpm+rev (#10)" },
	[MSET_OUT]    = { 40, 0,                        false, "convey->Laser3 rpm (#25)" },
	[MSET_EJECT]  = { 20, 0,                        false, "eject rpm (#30)" },
};

static bool valid(enum mset_id id)
{
	return (int)id >= 1 && (int)id <= MSET_COUNT;
}

uint32_t mset_rpm(enum mset_id id)        { return valid(id) ? rows[id].rpm : 0; }
uint32_t mset_steps(enum mset_id id)      { return valid(id) ? rows[id].steps : 0; }
bool     mset_has_rounds(enum mset_id id) { return valid(id) && rows[id].rounds; }
const char *mset_name(enum mset_id id)    { return valid(id) ? rows[id].name : "?"; }

int mset_set(enum mset_id id, uint32_t rpm, uint32_t steps)
{
	if (!valid(id) || rpm == 0) {
		return -EINVAL;
	}
	rows[id].rpm = rpm;
	if (rows[id].rounds) {
		rows[id].steps = steps;
	}
	return 0;
}

/* ---- persistence: one blob under "mset/v" ----------------------------- */
struct mset_blob {
	uint32_t rpm[MSET_COUNT];
	uint32_t steps[MSET_COUNT];
};

static int mset_settings_set(const char *name, size_t len, settings_read_cb read_cb,
			     void *cb_arg)
{
	const char *next;

	if (settings_name_steq(name, "v", &next) && !next) {
		struct mset_blob b;

		if (len != sizeof(b) || read_cb(cb_arg, &b, sizeof(b)) < 0) {
			return -EINVAL;
		}
		for (int id = 1; id <= MSET_COUNT; id++) {
			if (b.rpm[id - 1] > 0) {
				rows[id].rpm = b.rpm[id - 1];
			}
			if (rows[id].rounds) {
				rows[id].steps = b.steps[id - 1];
			}
		}
		return 0;
	}
	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(infeed_mset, "mset", NULL, mset_settings_set, NULL, NULL);

int mset_save(void)
{
	struct mset_blob b;
	int rc;

	for (int id = 1; id <= MSET_COUNT; id++) {
		b.rpm[id - 1] = rows[id].rpm;
		b.steps[id - 1] = rows[id].steps;
	}
	rc = settings_save_one("mset/v", &b, sizeof(b));
	if (rc) {
		LOG_ERR("mset save: %d", rc);
	}
	return rc;
}
