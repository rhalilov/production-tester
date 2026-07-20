#include "motion.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(motion, LOG_LEVEL_INF);

#define STEPS_PER_REV  ((uint64_t)MOTION_STEPS_PER_REV)

static bool invert[MOTION_AXIS_COUNT];   /* per-axis travel-sense (runtime) */

static const struct device *const axis_dev[MOTION_AXIS_COUNT] = {
	DEVICE_DT_GET(DT_NODELABEL(axis1)),
	DEVICE_DT_GET(DT_NODELABEL(axis2)),
	DEVICE_DT_GET(DT_NODELABEL(axis3)),
};

/* On-board LEDs (from the board DT): LED3=PD13, LED4=PD12, LED5=PD14. */
static const struct gpio_dt_spec led_m1   = GPIO_DT_SPEC_GET(DT_NODELABEL(orange_led_3), gpios);
static const struct gpio_dt_spec led_tpos = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_4), gpios);
static const struct gpio_dt_spec led_tneg = GPIO_DT_SPEC_GET(DT_NODELABEL(red_led_5), gpios);

static struct k_sem    done[MOTION_AXIS_COUNT];
static volatile bool   moving[MOTION_AXIS_COUNT];
static volatile int    cur_dir[MOTION_AXIS_COUNT];

static uint64_t interval_ns(uint32_t rpm)
{
	/* ns per pulse = 60e9 / (rpm * steps_per_rev) */
	return 60000000000ULL / ((uint64_t)rpm * STEPS_PER_REV);
}

static void on_event(const struct device *dev, enum stepper_event event, void *ud)
{
	ARG_UNUSED(dev);
	int axis = (int)(uintptr_t)ud;

	if (event == STEPPER_EVENT_STEPS_COMPLETED || event == STEPPER_EVENT_STOPPED) {
		moving[axis] = false;
		cur_dir[axis] = 0;
		k_sem_give(&done[axis]);
	}
}

int motion_init(void)
{
	int missing = 0;

	for (int i = 0; i < MOTION_AXIS_COUNT; i++) {
		k_sem_init(&done[i], 0, 1);
		if (!device_is_ready(axis_dev[i])) {
			LOG_ERR("axis%d not ready", i + 1);
			missing++;
			continue;
		}
		(void)stepper_set_event_callback(axis_dev[i], on_event,
						 (void *)(uintptr_t)i);
	}

	(void)gpio_pin_configure_dt(&led_m1,   GPIO_OUTPUT_INACTIVE);
	(void)gpio_pin_configure_dt(&led_tpos, GPIO_OUTPUT_INACTIVE);
	(void)gpio_pin_configure_dt(&led_tneg, GPIO_OUTPUT_INACTIVE);

	LOG_INF("motion up (%d axes, %d not-ready)", MOTION_AXIS_COUNT, missing);
	return missing ? -ENODEV : 0;
}

int motion_go_steps(enum motion_axis axis, enum motion_dir dir, uint32_t steps, uint32_t rpm)
{
	if (axis < 0 || axis >= MOTION_AXIS_COUNT || rpm == 0) {
		return -EINVAL;
	}
	const struct device *dev = axis_dev[axis];
	int err;

	k_sem_reset(&done[axis]);
	err = stepper_set_microstep_interval(dev, interval_ns(rpm));
	if (err) {
		return err;
	}
	int req  = (dir == MOTION_POS) ? 1 : -1;
	int phys = invert[axis] ? -req : req;       /* electrical sense */

	cur_dir[axis] = req;                        /* LEDs/motion_dir stay logical */
	moving[axis] = true;

	err = stepper_move_by(dev, (int32_t)steps * phys);
	if (err) {
		moving[axis] = false;
		cur_dir[axis] = 0;
		return err;
	}

	/* Backstop timeout = 2x expected move time + 2 s. */
	uint64_t exp_ms = (uint64_t)steps * 60000ULL / ((uint64_t)rpm * STEPS_PER_REV);

	if (k_sem_take(&done[axis], K_MSEC(exp_ms * 2 + 2000)) != 0) {
		(void)stepper_stop(dev);
		moving[axis] = false;
		cur_dir[axis] = 0;
		LOG_WRN("axis%d move timeout", axis + 1);
		return -ETIMEDOUT;
	}
	return 0;
}

int motion_go(enum motion_axis axis, enum motion_dir dir, uint32_t revs, uint32_t rpm)
{
	return motion_go_steps(axis, dir, revs * MOTION_STEPS_PER_REV, rpm);
}

int motion_run(enum motion_axis axis, enum motion_dir dir, uint32_t rpm)
{
	if (axis < 0 || axis >= MOTION_AXIS_COUNT || rpm == 0) {
		return -EINVAL;
	}
	const struct device *dev = axis_dev[axis];
	int err = stepper_set_microstep_interval(dev, interval_ns(rpm));

	if (err) {
		return err;
	}
	int req  = (dir == MOTION_POS) ? 1 : -1;
	int phys = invert[axis] ? -req : req;

	cur_dir[axis] = req;
	moving[axis] = true;
	return stepper_run(dev, (phys > 0) ? STEPPER_DIRECTION_POSITIVE
					   : STEPPER_DIRECTION_NEGATIVE);
}

void motion_stop(enum motion_axis axis)
{
	if (axis < 0 || axis >= MOTION_AXIS_COUNT) {
		return;
	}
	(void)stepper_stop(axis_dev[axis]);
	moving[axis] = false;
	cur_dir[axis] = 0;
}

void motion_stop_all(void)
{
	for (int i = 0; i < MOTION_AXIS_COUNT; i++) {
		motion_stop(i);
	}
}

bool motion_is_moving(enum motion_axis axis)
{
	if (axis < 0 || axis >= MOTION_AXIS_COUNT) {
		return false;
	}
	return moving[axis];
}

int motion_dir(enum motion_axis axis)
{
	if (axis < 0 || axis >= MOTION_AXIS_COUNT) {
		return 0;
	}
	return cur_dir[axis];
}

void motion_set_invert(enum motion_axis axis, bool inv)
{
	if (axis >= 0 && axis < MOTION_AXIS_COUNT) {
		invert[axis] = inv;
	}
}

bool motion_invert(enum motion_axis axis)
{
	return (axis >= 0 && axis < MOTION_AXIS_COUNT) ? invert[axis] : false;
}

void motion_leds_update(bool axis3_at_home)
{
	(void)gpio_pin_set_dt(&led_m1, moving[MOTION_AXIS1] ? 1 : 0);

	if (moving[MOTION_AXIS3] && cur_dir[MOTION_AXIS3] > 0) {
		(void)gpio_pin_set_dt(&led_tpos, 1);
		(void)gpio_pin_set_dt(&led_tneg, 0);
	} else if (moving[MOTION_AXIS3] && cur_dir[MOTION_AXIS3] < 0) {
		(void)gpio_pin_set_dt(&led_tpos, 0);
		(void)gpio_pin_set_dt(&led_tneg, 1);
	} else if (axis3_at_home) {
		(void)gpio_pin_set_dt(&led_tpos, 1);
		(void)gpio_pin_set_dt(&led_tneg, 1);
	} else {
		(void)gpio_pin_set_dt(&led_tpos, 0);
		(void)gpio_pin_set_dt(&led_tneg, 0);
	}
}
