#include "home.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(home, LOG_LEVEL_INF);

int home_init(void)
{
	/* TODO: per-axis homing against the limit/home sensors via io. */
	LOG_INF("home stub");
	return 0;
}
