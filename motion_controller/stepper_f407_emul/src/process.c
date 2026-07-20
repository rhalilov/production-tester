#include "process.h"
#include "io.h"
#include "roles.h"
#include "motion.h"
#include "solenoid.h"
#include "safety.h"
#include "evlog.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

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

/* Live "name pin" of the sensor currently bound to a role (for the event log). */
#define RS(r)  io_sensor_name(role_sensor(r)), io_sensor_pin(role_sensor(r))

/* Released by `infeed start` to leave the unconfigured safe-hold. */
static K_SEM_DEFINE(start_sem, 0, 1);

void process_start(void)
{
	k_sem_give(&start_sem);
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
		if (safety_in_fault()) {
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
		if (safety_in_fault()) {
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
	while (!io_smema_upstream_board_available()) {
		if (safety_in_fault()) {
			return false;
		}
		k_msleep(POLL_MS);
	}
	return true;
}

static bool wait_down_machine_ready(void)
{
	while (!io_smema_downstream_machine_ready()) {
		if (safety_in_fault()) {
			return false;
		}
		k_msleep(POLL_MS);
	}
	return true;
}

/* Blocking moves + fault check. Return false if faulted (caller aborts). */
static bool go(enum motion_axis ax, enum motion_dir d, uint32_t rev, uint32_t rpm)
{
	(void)motion_go(ax, d, rev, rpm);
	return !safety_in_fault();
}

static bool go_steps(enum motion_axis ax, enum motion_dir d, uint32_t steps, uint32_t rpm)
{
	(void)motion_go_steps(ax, d, steps, rpm);
	return !safety_in_fault();
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
		motion_run(MOTION_AXIS3, MOTION_POS, HOME_RPM);
		if (!wait_role(ROLE_TABLE_HOME)) {
			motion_stop(MOTION_AXIS3);
			return false;
		}
		motion_stop(MOTION_AXIS3);
	}
	evlog(1, '<', "Table home reached (up) - [table_home=%s %s] closed", RS(ROLE_TABLE_HOME));
	return true;
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

	enable_all_motors();
	selftest_bob();
	home_table();

	while (1) {
		if (safety_in_fault()) {
			k_msleep(200);
			continue;
		}

		/* #2 idle */
		evlog(2, '.', "Idle: waiting for upstream SMEMA Board Available (PC1)");
		if (!wait_up_board_available()) {
			continue;
		}

		/* #3 accept the board, start the cycle clock */
		cycle++;
		io_smema_set_upstream_machine_ready(true);
		evlog_cycle_start(cycle);
		evlog(3, '>', "Upstream Ready HIGH (PC2). Waiting for panel presented [panel_presented=%s %s]", RS(ROLE_PANEL_PRESENTED));

		/* #4 panel presented */
		if (!wait_role(ROLE_PANEL_PRESENTED)) {
			goto abort;
		}
		evlog(4, '<', "Panel presented at infeed - [panel_presented=%s %s] closed", RS(ROLE_PANEL_PRESENTED));
		io_smema_set_upstream_machine_ready(false);

		/* #5 conveyor infeed at 40 rpm — stopper stays DOWN so the belt can move */
		step_gap();
		motion_run(MOTION_AXIS1, MOTION_POS, 40);
		evlog(5, '>', "Motor 1 rotating positive 40 rpm infeed, stopper down (PLS-1 PE5). Waiting for [panel_near=%s %s]", RS(ROLE_PANEL_NEAR));
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

		/* #9 creep the panel 1/2 rev into the stopper, then stop */
		step_gap();
		if (!go_steps(MOTION_AXIS1, MOTION_POS, CREEP_STEPS, 10)) {
			goto abort;
		}
		evlog(9, '>', "Motor 1 creep positive 10 rpm 1/2 rev into stopper then stop (PLS-1 PE5)");

		/* #10 table DOWN to test position: negative 21 rev (20@40 + 1@10) */
		step_gap();
		if (!go(MOTION_AXIS3, MOTION_NEG, 20, 40) ||
		    !go(MOTION_AXIS3, MOTION_NEG, 1, 10)) {
			goto abort;
		}
		evlog(10, '>', "Table DOWN to test: motors 3+4 negative 21 rev (20@40 + 1@10) (PLS-3 PC8)");

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
		solenoid_set(SOLENOID3, SOLENOID_A);
		evlog(14, '>', "Arm panel locker: Solenoid 3 coil 1 pulse 60ms (PD10)");
		if (!confirm(15, ROLE_CYL3_UP, "Panel locker armed")) {
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

		/* #18 table UP to home: positive 20@40 then creep until table_home */
		step_gap();
		if (!go(MOTION_AXIS3, MOTION_POS, 20, 40)) {
			goto abort;
		}
		motion_run(MOTION_AXIS3, MOTION_POS, 10);
		evlog(18, '>', "Table UP to home: motors 3+4 positive 20 rev then creep until table home (PLS-3 PC8). Waiting for [table_home=%s %s]", RS(ROLE_TABLE_HOME));
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
		solenoid_set(SOLENOID3, SOLENOID_B);
		evlog(21, '>', "Release panel locker: Solenoid 3 coil 2 pulse 60ms (PD11)");
		if (!confirm(22, ROLE_CYL3_DOWN, "Panel locker released")) {
			goto abort;
		}

		/* #23 disarm stopper (gate down — releases the panel), #24 confirm down */
		step_gap();
		solenoid_set(SOLENOID1, SOLENOID_B);
		evlog(23, '>', "Disarm stopper: Solenoid 1 coil 2 pulse 60ms (PB13)");
		if (!confirm(24, ROLE_CYL1_DOWN, "Stopper disarmed")) {
			goto abort;
		}

		/* #25 conveyor to outfeed */
		step_gap();
		motion_run(MOTION_AXIS1, MOTION_POS, 40);
		evlog(25, '>', "Motor 1 rotating positive 40 rpm to outfeed (PLS-1 PE5). Waiting for [panel_in_position=%s %s]", RS(ROLE_PANEL_IN_POSITION));
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
		evlog(28, '>', "Offer panel downstream: SMEMA Board Available HIGH (PC3). Waiting for downstream Machine Ready (PA1)");
		/* #29 */
		if (!wait_down_machine_ready()) {
			goto abort;
		}
		evlog(29, '<', "Downstream ready - SMEMA Machine Ready (PA1) closed");

		/* #30 eject */
		step_gap();
		if (!go(MOTION_AXIS1, MOTION_POS, 20, 20)) {
			goto abort;
		}
		evlog(30, '>', "Eject panel: Motor 1 rotating positive 20 rev at 20 rpm (PLS-1 PE5)");
		io_smema_set_downstream_board_available(false);

		evlog_cycle_end(cycle);
		continue;

abort:
		motion_stop_all();
		io_smema_set_upstream_machine_ready(false);
		io_smema_set_downstream_board_available(false);
		evlog(0, '!', "ABORTED (fault): motion stopped, ENA released. Run 'infeed reset' to recover");
		while (safety_in_fault()) {
			k_msleep(100);
		}
		enable_all_motors();
		home_table();
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
