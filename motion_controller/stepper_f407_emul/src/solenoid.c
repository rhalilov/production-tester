#include "solenoid.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(solenoid, LOG_LEVEL_INF);

#define PULSE_MS 60   /* SY3220 response ~20 ms; 60 ms gives margin */

/* [valve][coil] — coil 0 = A, coil 1 = B. Driver+flyback are on the fixture;
 * MCU pin HIGH = energize (ACTIVE_HIGH in the overlay; polarity TBD at bench). */
static const struct gpio_dt_spec coil[SOLENOID_COUNT][2] = {
	{ GPIO_DT_SPEC_GET(DT_NODELABEL(sol1_a), gpios),
	  GPIO_DT_SPEC_GET(DT_NODELABEL(sol1_b), gpios) },
	{ GPIO_DT_SPEC_GET(DT_NODELABEL(sol2_a), gpios),
	  GPIO_DT_SPEC_GET(DT_NODELABEL(sol2_b), gpios) },
	{ GPIO_DT_SPEC_GET(DT_NODELABEL(sol3_a), gpios),
	  GPIO_DT_SPEC_GET(DT_NODELABEL(sol3_b), gpios) },
};

static struct k_work_delayable off_work[SOLENOID_COUNT];

static void coils_off(enum solenoid_id s)
{
	(void)gpio_pin_set_dt(&coil[s][0], 0);
	(void)gpio_pin_set_dt(&coil[s][1], 0);
}

static void off_handler(struct k_work *w)
{
	struct k_work_delayable *dw = k_work_delayable_from_work(w);
	int s = (int)(dw - off_work);

	coils_off(s);
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
		k_work_init_delayable(&off_work[s], off_handler);
	}
	LOG_INF("solenoids up (%d double valves)", SOLENOID_COUNT);
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

	/* Never both coils: drop any pending release, both off, then pulse. */
	(void)k_work_cancel_delayable(&off_work[s]);
	coils_off(s);
	(void)gpio_pin_set_dt(&coil[s][pos], 1);
	(void)k_work_schedule(&off_work[s], K_MSEC(PULSE_MS));
	return 0;
}

void solenoid_all_off(void)
{
	for (int s = 0; s < SOLENOID_COUNT; s++) {
		(void)k_work_cancel_delayable(&off_work[s]);
		coils_off(s);
	}
}
