/*
 * INFEED multi-axis stepper controller — entry point.
 *
 * Thin by design: just bring the modules up in order and hand off to the
 * supervisor thread (started by safety_init). All behaviour lives in the
 * modules: io (pins/EXTI), motion (3 axes), home, smema, safety (supervisor).
 */

#include "io.h"
#include "roles.h"
#include "motion.h"
#include "home.h"
#include "smema.h"
#include "solenoid.h"
#include "safety.h"
#include "process.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
	int err;

	LOG_INF("INFEED multi-axis controller starting");

	err = io_init();
	if (err) {
		LOG_ERR("io_init: %d", err);
	}
	err = roles_init();   /* sensor-role map + polarity (loads persisted config) */
	if (err) {
		LOG_ERR("roles_init: %d", err);
	}
	err = motion_init();
	if (err) {
		LOG_ERR("motion_init: %d", err);
	}
	err = home_init();
	if (err) {
		LOG_ERR("home_init: %d", err);
	}
	err = smema_init();
	if (err) {
		LOG_ERR("smema_init: %d", err);
	}
	err = solenoid_init();
	if (err) {
		LOG_ERR("solenoid_init: %d", err);
	}

	/* Supervisor thread (ALM/fault, LEDs, watchdog) — after all I/O is set. */
	err = safety_init();
	if (err) {
		LOG_ERR("safety_init: %d", err);
	}

	/* Last: the operation sequence (homing + per-panel cycle). */
	err = process_init();
	if (err) {
		LOG_ERR("process_init: %d", err);
	}

	LOG_INF("init complete");
	return 0;
}
