/*
 * io — the pin layer. Owns every fixture GPIO: the 15 interrupt inputs
 * (4 ALM + 11 sensors), the polled inputs (4 PEND + 2 SMEMA), and the SMEMA
 * outputs. The ISR only wakes the supervisor (semaphore); io_refresh() does the
 * debounced sampling. Passive: knows pins, holds no policy.
 */
#ifndef INFEED_IO_H_
#define INFEED_IO_H_

#include <stdbool.h>
#include <zephyr/kernel.h>

enum io_sensor_id {
	IO_LASER1, IO_LASER2, IO_LASER3,
	IO_INDUCTIVE4, IO_INDUCTIVE5, IO_INDUCTIVE6,
	IO_INDUCTIVE7, IO_INDUCTIVE8, IO_INDUCTIVE9,
	IO_PHOTO10, IO_PHOTO11,
	IO_SENSOR_COUNT
};

enum io_drive_id { IO_DRIVE1, IO_DRIVE2, IO_DRIVE3, IO_DRIVE4, IO_DRIVE_COUNT };

/* Configure all pins, register the per-port interrupt callbacks, enable EXTI. */
int io_init(void);

/* Block until an interrupt edge (ALM/sensor) or timeout. 0 = event, <0 = timeout. */
int io_wait_event(k_timeout_t timeout);

/* Re-sample + debounce every input, log transitions. Call from the supervisor. */
void io_refresh(void);

/* Debounced logical state (true = asserted). */
bool io_sensor(enum io_sensor_id s);
bool io_alm(enum io_drive_id d);
bool io_pend(enum io_drive_id d);

/* Per-sensor active level (runtime-configurable, persisted by the roles layer).
 * true = asserted-HIGH (NO sensor); false = asserted-LOW (NC). Applied to the
 * raw pin on the next io_refresh(). */
void io_set_active_high(enum io_sensor_id s, bool active_high);
bool io_active_high(enum io_sensor_id s);

/* Static identity helpers (fixed by hardware). */
const char *io_sensor_name(enum io_sensor_id s);
const char *io_sensor_pin(enum io_sensor_id s);

/* Simulation (logic test): force an input's logical state from the console, so
 * the sequence can be walked before real sensors are wired. RAM-only; forced
 * inputs ignore their physical pin until io_sim_off() reverts to live reads. */
void io_sim_sensor(enum io_sensor_id s, bool on);
void io_sim_smema_up_board_available(bool on);
void io_sim_smema_down_machine_ready(bool on);
void io_sim_off(void);
bool io_sim_any(void);

/* SMEMA (reserved) */
bool io_smema_upstream_board_available(void);   /* input  */
bool io_smema_downstream_machine_ready(void);   /* input  */
void io_smema_set_upstream_machine_ready(bool on);          /* output */
void io_smema_set_downstream_board_available(bool on);      /* output */
void io_smema_set_downstream_board_available_fail(bool on); /* output: FAIL board to next machine (PA3) */

#endif /* INFEED_IO_H_ */
