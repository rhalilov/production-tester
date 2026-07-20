#include "safety.h"
#include "io.h"
#include "roles.h"
#include "motion.h"
#include "solenoid.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(safety, LOG_LEVEL_INF);

#define POLL_MS  5

/*
 * Motor ENABLE lines (gpio-leds children in the overlay, GPIO_ACTIVE_LOW).
 * gpio_pin_set_dt(&ena, 1) => ENABLED; 0 => FREE.
 */
static const struct gpio_dt_spec ena[] = {
	GPIO_DT_SPEC_GET(DT_NODELABEL(ena1), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(ena2), gpios),
	GPIO_DT_SPEC_GET(DT_NODELABEL(ena3), gpios),
};

/* Discovery user button B1 (PA0) = E-STOP. Polled in the supervisor — its
 * EXTI line 0 is taken by Laser 3 (PB0), so we read the level, no interrupt. */
static const struct gpio_dt_spec estop_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

static const struct device *const wdt = DEVICE_DT_GET(DT_NODELABEL(iwdg));
static int  wdt_ch = -1;
static bool fault;

/* ALM faults are only honored AFTER every ALM line has been seen idle once —
 * the fixture's open-collector feedback settles slowly and variably at boot, so
 * we wait for "all clear" to arm rather than a fixed grace. */
static bool alm_armed;
static bool ena_state[ARRAY_SIZE(ena)];

BUILD_ASSERT(ARRAY_SIZE(ena) == SAFETY_AXES);

bool safety_in_fault(void)
{
	return fault;
}

void safety_fault_reset(void)
{
	fault = false;
	LOG_INF("fault cleared (motors remain free)");
}

static void set_all_enable(bool on)
{
	for (int i = 0; i < (int)ARRAY_SIZE(ena); i++) {
		(void)gpio_pin_set_dt(&ena[i], on ? 1 : 0);
		ena_state[i] = on;
	}
}

int safety_set_enable(int axis, bool on)
{
	if (axis < 0 || axis >= (int)ARRAY_SIZE(ena)) {
		return -EINVAL;
	}
	if (on && fault) {
		return -EBUSY;
	}

	int err = gpio_pin_set_dt(&ena[axis], on ? 1 : 0);

	if (err) {
		return err;
	}
	ena_state[axis] = on;
	return 0;
}

bool safety_enabled(int axis)
{
	if (axis < 0 || axis >= (int)ARRAY_SIZE(ena)) {
		return false;
	}
	return ena_state[axis];
}

static void enter_fault(const char *why)
{
	if (fault) {
		return;
	}
	fault = true;
	motion_stop_all();
	set_all_enable(false);
	solenoid_all_off();   /* bistable valves HOLD position when de-energized */
	LOG_ERR("FAULT latched: %s", why);
}

void safety_raise_fault(const char *why)
{
	enter_fault(why);
}

static void supervisor(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		/* Wake on any interrupt edge, else tick every POLL_MS to sample
		 * PEND/SMEMA and feed the watchdog. */
		(void)io_wait_event(K_MSEC(POLL_MS));
		io_refresh();

		/* E-STOP: user button latches the fault (motors freed for hand-moving). */
		if (gpio_pin_get_dt(&estop_btn) > 0) {
			enter_fault("E-STOP (user button)");
		}

		bool any_alm = false;

		for (int d = 0; d < IO_DRIVE_COUNT; d++) {
			if (io_alm(d)) {
				any_alm = true;
				break;
			}
		}
		if (!alm_armed) {
			if (!any_alm) {
				alm_armed = true;   /* lines settled idle -> arm */
			}
		} else if (any_alm) {
			enter_fault("drive alarm");
		}

		/* Motion indicator LEDs: table at home = table_home role + axis3 idle. */
		motion_leds_update(role_read(ROLE_TABLE_HOME) &&
				   !motion_is_moving(MOTION_AXIS3));

		if (wdt_ch >= 0) {
			(void)wdt_feed(wdt, wdt_ch);
		}
	}
}

K_THREAD_STACK_DEFINE(sup_stack, 2048);
static struct k_thread sup_thread;

int safety_init(void)
{
	for (int i = 0; i < (int)ARRAY_SIZE(ena); i++) {
		if (!gpio_is_ready_dt(&ena[i])) {
			LOG_ERR("ena%d not ready", i + 1);
			return -ENODEV;
		}
		/* Start FREE (motor disabled). */
		(void)gpio_pin_configure_dt(&ena[i], GPIO_OUTPUT_INACTIVE);
	}

	if (gpio_is_ready_dt(&estop_btn)) {
		(void)gpio_pin_configure_dt(&estop_btn, GPIO_INPUT);
	}

	/* IWDG ~500 ms, fed by the supervisor (<=POLL_MS loop). Pauses on a
	 * debugger halt so single-stepping does not reset the SoC. */
	if (device_is_ready(wdt)) {
		struct wdt_timeout_cfg cfg = {
			.window = { .min = 0U, .max = 500U },
			.callback = NULL,
			.flags = WDT_FLAG_RESET_SOC,
		};

		wdt_ch = wdt_install_timeout(wdt, &cfg);
		if (wdt_ch < 0) {
			LOG_WRN("wdt install: %d", wdt_ch);
		} else if (wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG)) {
			LOG_WRN("wdt setup failed");
			wdt_ch = -1;
		}
	} else {
		LOG_WRN("iwdg not ready");
	}

	k_thread_create(&sup_thread, sup_stack, K_THREAD_STACK_SIZEOF(sup_stack),
			supervisor, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	k_thread_name_set(&sup_thread, "supervisor");

	LOG_INF("safety up (%d ena, wdt_ch=%d)", (int)ARRAY_SIZE(ena), wdt_ch);
	return 0;
}
