#include "solenoid.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(solenoid, LOG_LEVEL_INF);

/* [valve][coil] — coil 0 = A, coil 1 = B. Driver+flyback are on the fixture;
 * MCU pin HIGH = energize (ACTIVE_HIGH in the overlay). LEVEL-driven: the
 * selected coil is held energized and the opposite de-energized until the next
 * solenoid_set(); never both energized at once. */
static const struct gpio_dt_spec coil[SOLENOID_COUNT][2] = {
	{ GPIO_DT_SPEC_GET(DT_NODELABEL(sol1_a), gpios),
	  GPIO_DT_SPEC_GET(DT_NODELABEL(sol1_b), gpios) },
	{ GPIO_DT_SPEC_GET(DT_NODELABEL(sol2_a), gpios),
	  GPIO_DT_SPEC_GET(DT_NODELABEL(sol2_b), gpios) },
	{ GPIO_DT_SPEC_GET(DT_NODELABEL(sol3_a), gpios),
	  GPIO_DT_SPEC_GET(DT_NODELABEL(sol3_b), gpios) },
};

static void coils_off(enum solenoid_id s)
{
	(void)gpio_pin_set_dt(&coil[s][0], 0);
	(void)gpio_pin_set_dt(&coil[s][1], 0);
}

int solenoid_init(void)
{
	for (int s = 0; s < SOLENOID_COUNT; s++) {
		for (int c = 0; c < 2; c++) {
			if (!gpio_is_ready_dt(&coil[s][c])) {
				LOG_ERR("sol%d coil%c not ready", s + 1, 'A' + c);
				return -ENODEV;
			}
			(void)gpio_pin_configure_dt(&coil[s][c], GPIO_OUTPUT_INACTIVE);
		}
	}
	LOG_INF("solenoids up (%d double valves, level-driven)", SOLENOID_COUNT);
	return 0;
}

int solenoid_set(enum solenoid_id s, enum solenoid_pos pos)
{
	if (s < 0 || s >= SOLENOID_COUNT) {
		return -EINVAL;
	}
	if (pos != SOLENOID_A && pos != SOLENOID_B) {
		return -EINVAL;
	}

	/* Level: drop the opposite coil first (never both), then HOLD the
	 * selected coil energized until the next call. */
	(void)gpio_pin_set_dt(&coil[s][pos ^ 1], 0);
	(void)gpio_pin_set_dt(&coil[s][pos], 1);
	return 0;
}

void solenoid_all_off(void)
{
	for (int s = 0; s < SOLENOID_COUNT; s++) {
		coils_off(s);
	}
}
