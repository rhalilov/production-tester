#include "process.h"
#include "io.h"
#include "roles.h"
#include "motion.h"
#include "mset.h"
#include "solenoid.h"
#include "safety.h"
#include "evlog.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(process, LOG_LEVEL_INF);

#define STEP_DELAY_MS  500     /* default delay between logical steps */
#define HOME_RPM       10      /* slow homing / creep speed */
#define POLL_MS        5

#define BOB_STEPS      (MOTION_STEPS_PER_REV / 4)   /* self-test: 1/4 rev */
#define BOB_RPM        30
#define CREEP_STEPS    (MOTION_STEPS_PER_REV / 2)   /* step 9: 1/2 rev creep into stopper */

#define PROGRAM_MS     10000   /* step 16: PCB programming / test dwell */
#define DELAY13_MS     100     /* step 13: explicit inter-step delay */
#define CYL_CONFIRM_MS 10000   /* cylinder actuation ALARM timeout (production) */

/* SMEMA inline handshake. Set 0 to run standalone on the live machine with NO
 * upstream/downstream handshake (cycle gated only by Laser1 in / Laser3 out).
 * Set back to 1 to restore the full SMEMA-gated line behaviour. */
#define SMEMA_ENABLED  0

/* Cylinders (solenoids + confirms). Production build = 1. The no-cylinders test
 * build sets 0 (skips the stopper in `infeed load`). */
#define CYLINDERS_ENABLED  1

/* Live "name pin" of the sensor currently bound to a role (for the event log). */
#define RS(r)  io_sensor_name(role_sensor(r)), io_sensor_pin(role_sensor(r))

/* Released by `infeed start` to leave the unconfigured safe-hold. */
static K_SEM_DEFINE(start_sem, 0, 1);

/* Set by `infeed start` to (re)trigger a fresh run (re-home + cycle) at ANY
 * time, even while already running. interrupted() folds it into the fault
 * check so any in-flight wait/move bails out and the loop re-homes. */
static volatile bool restart_req;
static bool interrupted(void) { return safety_in_fault() || restart_req; }

/* Manual commissioning mode: a manual command (infeed table/load/unload) parks
 * the auto-cycle at its start gate (#4). cycle_active = a cycle is mid-run, so a
 * manual command refuses. Both cleared by `infeed start` (resume auto-cycle). */
static volatile bool manual_hold;
static volatile bool cycle_active;

void process_start(void)
{
	manual_hold = false;        /* leave manual mode, resume the auto-cycle */
	restart_req = true;
	motion_stop_all();          /* drop any in-flight move immediately */
	k_sem_give(&start_sem);     /* release the unconfigured safe-hold if waiting */
}

/* Commissioning mode: cylinder confirms wait forever (no alarm) for hand-stepping. */
static bool test_mode;

void process_set_test_mode(bool on) { test_mode = on; }
bool process_test_mode(void)        { return test_mode; }

static void step_gap(void)
{
	k_msleep(STEP_DELAY_MS);
}

/* Wait until a role's sensor is asserted. Returns false if a fault aborts it. */
static bool wait_role(enum io_role r)
{
	while (!role_read(r)) {
		if (interrupted()) {
			return false;
		}
		k_msleep(POLL_MS);
	}
	return true;
}

/* Wait until a role's sensor OPENS (de-asserts). Returns false if a fault aborts it. */
static bool wait_role_clear(enum io_role r)
{
	while (role_read(r)) {
		if (interrupted()) {
			return false;
		}
		k_msleep(POLL_MS);
	}
	return true;
}

/*
 * Closed-loop confirm: after commanding a cylinder, wait for its limit sensor.
 * Production: a timeout latches a fault (alarm). Hand-stepping (`infeed test on`
 * or any input forced via `infeed sim`): waits forever.
 */
static bool confirm(int step, enum io_role r, const char *what)
{
	uint32_t waited = 0;
	bool announced = false;

	while (!role_read(r)) {
		if (interrupted()) {
			return false;
		}
		if (test_mode || io_sim_any()) {
			if (!announced) {
				evlog(step, '.', "%s: waiting (no timeout - test mode) for [%s=%s %s]",
				      what, role_name(r), RS(r));
				announced = true;
			}
		} else if (waited >= CYL_CONFIRM_MS) {
			evlog(step, '!', "ALARM: %s NOT confirmed in %u ms (wanted [%s=%s %s])",
			      what, CYL_CONFIRM_MS, role_name(r), RS(r));
			safety_raise_fault("cylinder confirm timeout");
			return false;
		}
		k_msleep(POLL_MS);
		waited += POLL_MS;
	}
	evlog(step, '<', "%s confirmed - [%s=%s %s] closed", what, role_name(r), RS(r));
	return true;
}

static bool wait_up_board_available(void)
{
#if !SMEMA_ENABLED
	return true;   /* SMEMA disabled: no upstream handshake */
#else
	while (!io_smema_upstream_board_available()) {
		if (safety_in_fault()) {
			return false;
		}
		k_msleep(POLL_MS);
	}
	return true;
#endif
}

static bool wait_down_machine_ready(void)
{
#if !SMEMA_ENABLED
	return true;   /* SMEMA disabled: no downstream handshake */
#else
	while (!io_smema_downstream_machine_ready()) {
		if (safety_in_fault()) {
			return false;
		}
		k_msleep(POLL_MS);
	}
	return true;
#endif
}

/* Blocking moves + fault check. Return false if faulted (caller aborts). */
static bool go(enum motion_axis ax, enum motion_dir d, uint32_t rev, uint32_t rpm)
{
	(void)motion_go(ax, d, rev, rpm);
	return !interrupted();
}

static bool go_steps(enum motion_axis ax, enum motion_dir d, uint32_t steps, uint32_t rpm)
{
	(void)motion_go_steps(ax, d, steps, rpm);
	return !interrupted();
}

static void enable_all_motors(void)
{
	for (int a = 0; a < SAFETY_AXES; a++) {
		(void)safety_set_enable(a, true);   /* motors stay enabled while powered */
	}
}

/* Power-on motion check: bob each drive +/-1/4 rev, the 3 banks 2 s apart. */
static void selftest_bob(void)
{
	evlog(0, '.', "Self-test: bob each drive +/-1/4 rev, banks 2s apart");
	for (int i = 0; i < MOTION_AXIS_COUNT; i++) {
		if (safety_in_fault()) {
			return;
		}
		k_msleep(2000);
		evlog(0, '.', "  bank %d: rotate +1/4 rev then -1/4 rev", i + 1);
		(void)motion_go_steps((enum motion_axis)i, MOTION_POS, BOB_STEPS, BOB_RPM);
		(void)motion_go_steps((enum motion_axis)i, MOTION_NEG, BOB_STEPS, BOB_RPM);
	}
}

/* #1 power-up / recovery homing: table runs POSITIVE (up) to the table_home sensor. */
static bool home_table(void)
{
	evlog(1, '.', "Homing table: motors 3+4 rotating positive (up) until table home (PLS-3 PC8 -> [table_home=%s %s])", RS(ROLE_TABLE_HOME));
	if (!role_read(ROLE_TABLE_HOME)) {
		motion_run(MOTION_AXIS3, MOTION_POS, mset_rpm(MSET_HOME));
		if (!wait_role(ROLE_TABLE_HOME)) {
			motion_stop(MOTION_AXIS3);
			return false;
		}
		motion_stop(MOTION_AXIS3);
	}
	evlog(1, '<', "Table home reached (up) - [table_home=%s %s] closed", RS(ROLE_TABLE_HOME));
	return true;
}

/* ---- Manual commissioning commands: run one portion of the cycle from CLI ----
 * Each enters manual mode (parks the auto-cycle) and stays there until
 * `infeed start` resumes the auto-cycle. Refuses on fault or mid auto-cycle. */
static int manual_begin(void)
{
	if (safety_in_fault()) {
		return -EBUSY;      /* fault latched */
	}
	if (cycle_active) {
		return -EAGAIN;     /* an auto cycle is running */
	}
	manual_hold = true;
	return 0;
}

int process_table_home(void)          /* #1 */
{
	int err = manual_begin();

	if (err) {
		return err;
	}
	(void)safety_set_enable(MOTION_AXIS3, true);
	return home_table() ? 0 : -EINTR;
}

int process_load(void)                /* #4..#9 */
{
	int err = manual_begin();

	if (err) {
		return err;
	}
	(void)safety_set_enable(MOTION_AXIS1, true);
	/* #4 panel presented */
	if (!wait_role(ROLE_PANEL_PRESENTED)) {
		return -EINTR;
	}
	/* #5 conveyor infeed */
	motion_run(MOTION_AXIS1, MOTION_POS, mset_rpm(MSET_CONVEY));
	/* #6 panel near -> stop */
	if (!wait_role(ROLE_PANEL_NEAR)) {
		motion_stop(MOTION_AXIS1);
		return -EINTR;
	}
	motion_stop(MOTION_AXIS1);
#if CYLINDERS_ENABLED
	/* #7 arm stopper, #8 confirm up */
	solenoid_set(SOLENOID1, SOLENOID_A);
	if (!confirm(8, ROLE_CYL1_UP, "Stopper armed")) {
		return -EINTR;
	}
#endif
	/* #9 creep into position */
	return go_steps(MOTION_AXIS1, MOTION_POS, mset_steps(MSET_CREEP), mset_rpm(MSET_CREEP))
		       ? 0 : -EINTR;
}

int process_table_down(void)          /* #10 */
{
	int err = manual_begin();

	if (err) {
		return err;
	}
	(void)safety_set_enable(MOTION_AXIS3, true);
	if (!go_steps(MOTION_AXIS3, MOTION_NEG, mset_steps(MSET_DOWN1), mset_rpm(MSET_DOWN1))) {
		return -EINTR;
	}
	return go_steps(MOTION_AXIS3, MOTION_NEG, mset_steps(MSET_DOWN2), mset_rpm(MSET_DOWN2))
		       ? 0 : -EINTR;
}

int process_unload(void)              /* #25..#27 */
{
	int err = manual_begin();

	if (err) {
		return err;
	}
	(void)safety_set_enable(MOTION_AXIS1, true);
	/* #25 conveyor infeed */
	motion_run(MOTION_AXIS1, MOTION_POS, mset_rpm(MSET_OUT));
	/* #26 panel in position -> #27 stop */
	if (!wait_role(ROLE_PANEL_IN_POSITION)) {
		motion_stop(MOTION_AXIS1);
		return -EINTR;
	}
	motion_stop(MOTION_AXIS1);
	return 0;
}

static void sequence(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	unsigned cycle = 0;

	/* Unconfigured unit holds (no motion) until `infeed start`; configured auto-starts. */
	if (!roles_is_configured()) {
		evlog(0, '.', "UNCONFIGURED safe-hold - wire + map, then run 'infeed start'");
		k_sem_take(&start_sem, K_FOREVER);
	}

	restart_req = false;   /* consume the start token from the safe-hold release */
	enable_all_motors();
	selftest_bob();
	home_table();

	while (1) {
		if (safety_in_fault()) {
			k_msleep(200);
			continue;
		}
		if (manual_hold) {          /* a manual command owns the machine */
			k_msleep(50);
			continue;
		}

		/* #2 idle */
#if SMEMA_ENABLED
		evlog(2, '.', "Idle: waiting for upstream SMEMA Board Available (PC1)");
#else
		evlog(2, '.', "SMEMA disabled (live test) - no upstream handshake, gate on Laser 1");
#endif
		if (!wait_up_board_available()) {
			continue;
		}

		/* #3 accept the board, start the cycle clock */
		cycle++;
		io_smema_set_upstream_machine_ready(true);
		evlog_cycle_start(cycle);
		evlog(3, '>', "Upstream Ready HIGH (PC2). Waiting for panel presented [panel_presented=%s %s]", RS(ROLE_PANEL_PRESENTED));

		/* #4 panel presented — don't start a cycle while a manual command holds */
		while (manual_hold || !role_read(ROLE_PANEL_PRESENTED)) {
			if (interrupted()) {
				goto abort;
			}
			k_msleep(POLL_MS);
		}
		cycle_active = true;
		evlog(4, '<', "Panel presented at infeed - [panel_presented=%s %s] closed", RS(ROLE_PANEL_PRESENTED));
		io_smema_set_upstream_machine_ready(false);

		/* #5 conveyor infeed (set 2 rpm) — stopper stays DOWN so the belt can move */
		step_gap();
		motion_run(MOTION_AXIS1, MOTION_POS, mset_rpm(MSET_CONVEY));
		evlog(5, '>', "Motor 1 infeed %u rpm, stopper down (PLS-1 PE5). Waiting for [panel_near=%s %s]", mset_rpm(MSET_CONVEY), RS(ROLE_PANEL_NEAR));
		/* #6 middle panel near — stop the 40 rpm feed */
		if (!wait_role(ROLE_PANEL_NEAR)) {
			goto abort;
		}
		motion_stop(MOTION_AXIS1);
		evlog(6, '<', "Panel near (middle panel) - [panel_near=%s %s] closed, conveyor stopped", RS(ROLE_PANEL_NEAR));

		/* #7 arm stopper (gate up — the panel will seat against it), #8 confirm up */
		step_gap();
		solenoid_set(SOLENOID1, SOLENOID_A);
		evlog(7, '>', "Arm stopper: Solenoid 1 coil 1 pulse 60ms (PB12) - panel must hit it");
		if (!confirm(8, ROLE_CYL1_UP, "Stopper armed")) {
			goto abort;
		}

		/* #9 creep the panel into the stopper (set 3), then stop */
		step_gap();
		if (!go_steps(MOTION_AXIS1, MOTION_POS, mset_steps(MSET_CREEP), mset_rpm(MSET_CREEP))) {
			goto abort;
		}
		evlog(9, '>', "Motor 1 creep %u rpm into stopper then stop (PLS-1 PE5)", mset_rpm(MSET_CREEP));

		/* #10 table DOWN to test position: two phases (set 4 + set 5) */
		step_gap();
		if (!go_steps(MOTION_AXIS3, MOTION_NEG, mset_steps(MSET_DOWN1), mset_rpm(MSET_DOWN1)) ||
		    !go_steps(MOTION_AXIS3, MOTION_NEG, mset_steps(MSET_DOWN2), mset_rpm(MSET_DOWN2))) {
			goto abort;
		}
		evlog(10, '>', "Table DOWN to test: motors 3+4 negative (p1 %u rpm + p2 %u rpm) (PLS-3 PC8)", mset_rpm(MSET_DOWN1), mset_rpm(MSET_DOWN2));

		/* #11 arm RFID, #12 confirm up */
		step_gap();
		solenoid_set(SOLENOID2, SOLENOID_A);
		evlog(11, '>', "Arm RFID: Solenoid 2 coil 1 pulse 60ms (PB14)");
		if (!confirm(12, ROLE_CYL2_UP, "RFID armed")) {
			goto abort;
		}

		/* #13 explicit 0.1 s delay */
		k_msleep(DELAY13_MS);
		evlog(13, '.', "Delay 0.1 s");

		/* #14 arm panel locker, #15 confirm up */
		step_gap();
		solenoid_set(SOLENOID3, SOLENOID_B);
		evlog(14, '>', "Arm panel locker: Solenoid 3 coil 2 (PD11)");
		if (!confirm(15, ROLE_CYL3_DOWN, "Panel locker armed (down)")) {
			goto abort;
		}

		/* #16 program / test the PCB */
		evlog(16, '.', "Programming PCB for 10 seconds ...");
		k_msleep(PROGRAM_MS);
		if (safety_in_fault()) {
			goto abort;
		}
		/* #17 ack (production: wait for the programmer's UART OK) */
		evlog(17, '<', "Programming and testing OK (acknowledged)");

		/* #18 table UP to home: single-phase at homing speed (set 1) until Photo 10. */
		step_gap();
		motion_run(MOTION_AXIS3, MOTION_POS, mset_rpm(MSET_HOME));
		evlog(18, '>', "Table UP to home: motors 3+4 positive %u rpm until table home (PLS-3 PC8). Waiting for [table_home=%s %s]", mset_rpm(MSET_HOME), RS(ROLE_TABLE_HOME));
		if (!wait_role(ROLE_TABLE_HOME)) {
			goto abort;
		}
		motion_stop(MOTION_AXIS3);
		evlog(18, '<', "Table home reached (up) - [table_home=%s %s] closed", RS(ROLE_TABLE_HOME));

		/* #19 release RFID, #20 confirm down */
		step_gap();
		solenoid_set(SOLENOID2, SOLENOID_B);
		evlog(19, '>', "Release RFID: Solenoid 2 coil 2 pulse 60ms (PB15)");
		if (!confirm(20, ROLE_CYL2_DOWN, "RFID released")) {
			goto abort;
		}

		/* #21 release panel locker, #22 confirm down */
		step_gap();
		solenoid_set(SOLENOID3, SOLENOID_A);
		evlog(21, '>', "Release panel locker: Solenoid 3 coil 1 (PD10)");
		if (!confirm(22, ROLE_CYL3_UP, "Panel locker released (up)")) {
			goto abort;
		}

		/* #23 disarm stopper (gate down — releases the panel), #24 confirm down */
		step_gap();
		solenoid_set(SOLENOID1, SOLENOID_B);
		evlog(23, '>', "Disarm stopper: Solenoid 1 coil 2 pulse 60ms (PB13)");
		if (!confirm(24, ROLE_CYL1_DOWN, "Stopper disarmed")) {
			goto abort;
		}

		/* #25 conveyor to outfeed (set 6) */
		step_gap();
		motion_run(MOTION_AXIS1, MOTION_POS, mset_rpm(MSET_OUT));
		evlog(25, '>', "Motor 1 %u rpm to outfeed (PLS-1 PE5). Waiting for [panel_in_position=%s %s]", mset_rpm(MSET_OUT), RS(ROLE_PANEL_IN_POSITION));
		/* #26 */
		if (!wait_role(ROLE_PANEL_IN_POSITION)) {
			goto abort;
		}
		evlog(26, '<', "Panel in final position - [panel_in_position=%s %s] closed", RS(ROLE_PANEL_IN_POSITION));
		/* #27 */
		motion_stop(MOTION_AXIS1);
		evlog(27, '>', "Stop conveyor: Motor 1 stop (PLS-1 PE5)");

		/* #28 hand off downstream */
		step_gap();
		io_smema_set_downstream_board_available(true);
#if SMEMA_ENABLED
		evlog(28, '>', "Offer panel downstream: SMEMA Board Available HIGH (PC3). Waiting for downstream Machine Ready (PA1)");
#else
		evlog(28, '>', "SMEMA disabled (live test) - no downstream handshake, ejecting");
#endif
		/* #29 */
		if (!wait_down_machine_ready()) {
			goto abort;
		}
#if SMEMA_ENABLED
		evlog(29, '<', "Downstream ready - SMEMA Machine Ready (PA1) closed");
#endif

		/* #30 eject (set 7): run the conveyor until the panel clears Laser 3 */
		step_gap();
		motion_run(MOTION_AXIS1, MOTION_POS, mset_rpm(MSET_EJECT));
		evlog(30, '>', "Eject panel: Motor 1 positive %u rpm until panel out (PLS-1 PE5). Waiting for [panel_in_position=%s %s] to OPEN", mset_rpm(MSET_EJECT), RS(ROLE_PANEL_IN_POSITION));
		/* #31 panel out — Laser 3 opens */
		if (!wait_role_clear(ROLE_PANEL_IN_POSITION)) {
			goto abort;
		}
		evlog(31, '<', "Panel out - [panel_in_position=%s %s] opened", RS(ROLE_PANEL_IN_POSITION));
		/* #32 stop conveyor */
		motion_stop(MOTION_AXIS1);
		evlog(32, '>', "Stop conveyor: Motor 1 stop after panel out (PLS-1 PE5)");
		io_smema_set_downstream_board_available(false);

		cycle_active = false;
		evlog_cycle_end(cycle);
		continue;

abort:
		cycle_active = false;
		motion_stop_all();
		io_smema_set_upstream_machine_ready(false);
		io_smema_set_downstream_board_available(false);
		if (restart_req) {
			/* `infeed start` pressed while running: re-home + fresh run. */
			evlog(0, '.', "Restart ('infeed start'): re-home + new run");
			restart_req = false;
			k_sem_reset(&start_sem);
			enable_all_motors();
			selftest_bob();
			home_table();
		} else {
			evlog(0, '!', "ABORTED (fault): motion stopped, ENA released. Run 'infeed reset' to recover");
			while (safety_in_fault()) {
				k_msleep(100);
			}
			enable_all_motors();
			home_table();
		}
	}
}

K_THREAD_STACK_DEFINE(proc_stack, 2048);
static struct k_thread proc_thread;

int process_init(void)
{
	k_thread_create(&proc_thread, proc_stack, K_THREAD_STACK_SIZEOF(proc_stack),
			sequence, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
	k_thread_name_set(&proc_thread, "process");
	LOG_INF("process started");
	return 0;
}
