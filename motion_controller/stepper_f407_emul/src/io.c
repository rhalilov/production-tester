#include "io.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(io, LOG_LEVEL_INF);

#define ZU      DT_PATH(zephyr_user)
#define SPEC(p) GPIO_DT_SPEC_GET(ZU, p)

/*
 * Single flat input table. Order matters: sensors + ALM (the interrupt set)
 * come first and contiguous, so indices [0, N_IRQ) are exactly the EXTI inputs.
 */
#define N_SENSOR       IO_SENSOR_COUNT                 /* 11 */
#define N_DRIVE        IO_DRIVE_COUNT                  /* 4  */
#define I_SENSOR(s)    (s)                             /* 0..10  */
#define I_ALM(d)       (N_SENSOR + (d))                /* 11..14 */
#define I_PEND(d)      (N_SENSOR + N_DRIVE + (d))      /* 15..18 */
#define I_SMEMA_UP_BA  (N_SENSOR + 2 * N_DRIVE)        /* 19 */
#define I_SMEMA_DN_MR  (N_SENSOR + 2 * N_DRIVE + 1)    /* 20 */
#define N_INPUTS       (N_SENSOR + 2 * N_DRIVE + 2)    /* 21 */
#define N_IRQ          (N_SENSOR + N_DRIVE)            /* 15 (sensors + ALM) */

BUILD_ASSERT(N_INPUTS <= 32, "level bitmask is 32-bit");

#define DEBOUNCE_MS 2

static const struct gpio_dt_spec inputs[N_INPUTS] = {
	/* sensors 0..10 (interrupt): 3 laser, 6 inductive, 2 photo (cont. numbering) */
	SPEC(laser1_gpios),     SPEC(laser2_gpios),     SPEC(laser3_gpios),
	SPEC(inductive4_gpios), SPEC(inductive5_gpios), SPEC(inductive6_gpios),
	SPEC(inductive7_gpios), SPEC(inductive8_gpios), SPEC(inductive9_gpios),
	SPEC(photo10_gpios),    SPEC(photo11_gpios),
	/* ALM 11..14 (interrupt) */
	SPEC(alm1_gpios), SPEC(alm2_gpios), SPEC(alm3_gpios), SPEC(alm4_gpios),
	/* PEND 15..18 (polled) */
	SPEC(pend1_gpios), SPEC(pend2_gpios), SPEC(pend3_gpios), SPEC(pend4_gpios),
	/* SMEMA inputs 19..20 (polled) */
	SPEC(smema_up_board_available_gpios),
	SPEC(smema_down_machine_ready_gpios),
};

static const char *const names[N_INPUTS] = {
	"laser1", "laser2", "laser3",
	"inductive4", "inductive5", "inductive6",
	"inductive7", "inductive8", "inductive9",
	"photo10", "photo11",
	"alm1", "alm2", "alm3", "alm4",
	"pend1", "pend2", "pend3", "pend4",
	"smema_up_board_avail", "smema_down_mach_ready",
};

/* Pin-name strings for the 11 sensors (fixed by hardware; only the role binding
 * is reconfigurable). For human-readable CLI / event log. */
static const char *const sensor_pins[N_SENSOR] = {
	"PC5", "PB1", "PB0",                          /* laser1-3      */
	"PB2", "PE7", "PE9", "PE10", "PE12", "PE13",  /* inductive4-9  */
	"PE14", "PE15",                               /* photo10-11    */
};

/* Per-sensor active level (runtime/persisted). true = asserted-HIGH (NO). */
static bool sensor_active_high[N_SENSOR];

static const struct gpio_dt_spec smema_out[] = {
	SPEC(smema_up_machine_ready_gpios),
	SPEC(smema_down_board_available_gpios),
	SPEC(smema_down_board_available_fail_gpios),
};

static K_SEM_DEFINE(event_sem, 0, 1);

static uint32_t level;               /* debounced state, bit i = inputs[i] */
static uint32_t forced_mask;         /* bit i = inputs[i] is simulated (skip pin) */
static int64_t  last_edge[N_IRQ];    /* debounce timestamps (interrupt set) */

/* One callback per port that owns interrupt pins (the 15 span ports A/B/C/E). */
#define MAX_PORTS 6
static struct gpio_callback port_cb[MAX_PORTS];
static const struct device *cb_port[MAX_PORTS];
static int n_ports;

static void input_isr(const struct device *port, struct gpio_callback *cb,
		      gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	k_sem_give(&event_sem);
}

static int port_slot(const struct device *port)
{
	for (int i = 0; i < n_ports; i++) {
		if (cb_port[i] == port) {
			return i;
		}
	}
	if (n_ports >= MAX_PORTS) {
		return -1;
	}
	cb_port[n_ports] = port;
	return n_ports++;
}

/* Asserted (logical) level of input i. Sensors apply the runtime active-level
 * to the raw pin; ALM/PEND/SMEMA use their fixed devicetree polarity. */
static int read_asserted(int i)
{
	if (i < N_SENSOR) {
		int raw = gpio_pin_get_raw(inputs[i].port, inputs[i].pin);

		if (raw < 0) {
			return raw;
		}
		return sensor_active_high[i] ? !!raw : !raw;
	}
	return gpio_pin_get_dt(&inputs[i]);
}

int io_init(void)
{
	gpio_port_pins_t mask[MAX_PORTS] = {0};
	int err;

	for (int s = 0; s < N_SENSOR; s++) {
		sensor_active_high[s] = true;   /* most sensors default ACTIVE_HIGH */
	}
	/* Lasers are ACTIVE-LOW on the v0.2.0 board (asserted = pin pulled low). */
	sensor_active_high[IO_LASER1] = false;
	sensor_active_high[IO_LASER2] = false;
	sensor_active_high[IO_LASER3] = false;

	for (int i = 0; i < N_INPUTS; i++) {
		if (!gpio_is_ready_dt(&inputs[i])) {
			LOG_ERR("%s: port not ready", names[i]);
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&inputs[i], GPIO_INPUT);
		if (err) {
			LOG_ERR("%s: configure %d", names[i], err);
			return err;
		}
	}

	for (int i = 0; i < (int)ARRAY_SIZE(smema_out); i++) {
		if (!gpio_is_ready_dt(&smema_out[i])) {
			return -ENODEV;
		}
		(void)gpio_pin_configure_dt(&smema_out[i], GPIO_OUTPUT_INACTIVE);
	}

	/* Build per-port masks over the interrupt set, then register one callback
	 * per port (init needs the full mask before add). */
	for (int i = 0; i < N_IRQ; i++) {
		int slot = port_slot(inputs[i].port);

		if (slot < 0) {
			LOG_ERR("too many interrupt ports");
			return -ENOMEM;
		}
		mask[slot] |= BIT(inputs[i].pin);
	}
	for (int i = 0; i < n_ports; i++) {
		gpio_init_callback(&port_cb[i], input_isr, mask[i]);
		err = gpio_add_callback(cb_port[i], &port_cb[i]);
		if (err) {
			LOG_ERR("add_callback: %d", err);
			return err;
		}
	}

	/* Prime current state (so the first refresh only logs real changes). */
	for (int i = 0; i < N_INPUTS; i++) {
		if (read_asserted(i) > 0) {
			level |= BIT(i);
		}
	}

	/* Enable edge interrupts on the 15. Distinct EXTI lines were engineered
	 * into the pin map, so no -EBUSY collisions. */
	for (int i = 0; i < N_IRQ; i++) {
		err = gpio_pin_interrupt_configure_dt(&inputs[i], GPIO_INT_EDGE_BOTH);
		if (err) {
			LOG_ERR("%s: irq enable %d", names[i], err);
			return err;
		}
	}

	LOG_INF("io ready: %d inputs (%d irq on %d ports, %d polled)",
		N_INPUTS, N_IRQ, n_ports, N_INPUTS - N_IRQ);
	return 0;
}

int io_wait_event(k_timeout_t timeout)
{
	return k_sem_take(&event_sem, timeout);
}

void io_refresh(void)
{
	int64_t now = k_uptime_get();

	for (int i = 0; i < N_INPUTS; i++) {
		if (forced_mask & BIT(i)) {
			continue;          /* simulated input: don't overwrite */
		}

		int v = read_asserted(i);

		if (v < 0) {
			continue;
		}
		if ((bool)(level & BIT(i)) == (bool)v) {
			continue;
		}
		if (i < N_IRQ) {
			if (now - last_edge[i] < DEBOUNCE_MS) {
				continue; /* debounce re-fire (interrupt set only) */
			}
			last_edge[i] = now;
		}
		WRITE_BIT(level, i, v);
		LOG_INF("%s -> %d", names[i], v);
	}
}

bool io_sensor(enum io_sensor_id s) { return level & BIT(I_SENSOR(s)); }
bool io_alm(enum io_drive_id d)     { return level & BIT(I_ALM(d)); }
bool io_pend(enum io_drive_id d)    { return level & BIT(I_PEND(d)); }

bool io_smema_upstream_board_available(void) { return level & BIT(I_SMEMA_UP_BA); }
bool io_smema_downstream_machine_ready(void) { return level & BIT(I_SMEMA_DN_MR); }

void io_smema_set_upstream_machine_ready(bool on)
{
	(void)gpio_pin_set_dt(&smema_out[0], on);
}
void io_smema_set_downstream_board_available(bool on)
{
	(void)gpio_pin_set_dt(&smema_out[1], on);
}

void io_smema_set_downstream_board_available_fail(bool on)
{
	(void)gpio_pin_set_dt(&smema_out[2], on);
}

void io_set_active_high(enum io_sensor_id s, bool active_high)
{
	if ((int)s < N_SENSOR) {
		sensor_active_high[s] = active_high;
	}
}

bool io_active_high(enum io_sensor_id s)
{
	return ((int)s < N_SENSOR) ? sensor_active_high[s] : true;
}

const char *io_sensor_name(enum io_sensor_id s)
{
	return ((int)s < N_SENSOR) ? names[s] : "?";
}

const char *io_sensor_pin(enum io_sensor_id s)
{
	return ((int)s < N_SENSOR) ? sensor_pins[s] : "?";
}

static void sim_set(int i, bool on)
{
	forced_mask |= BIT(i);
	WRITE_BIT(level, i, on);
	LOG_INF("SIM %s = %d", names[i], on ? 1 : 0);
}

void io_sim_sensor(enum io_sensor_id s, bool on)
{
	if ((int)s < N_SENSOR) {
		sim_set(I_SENSOR(s), on);
	}
}

void io_sim_smema_up_board_available(bool on)   { sim_set(I_SMEMA_UP_BA, on); }
void io_sim_smema_down_machine_ready(bool on)   { sim_set(I_SMEMA_DN_MR, on); }

void io_sim_off(void)
{
	forced_mask = 0;       /* next refresh re-reads the live pins */
	LOG_INF("SIM off (live inputs)");
}

bool io_sim_any(void) { return forced_mask != 0; }
