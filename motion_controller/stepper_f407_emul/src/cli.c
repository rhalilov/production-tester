/*
 * cli - the `infeed` shell (bring-up / bench tool). Pure glue: reads io/safety
 * state and formats it. Raw per-axis motion stays on the stock `stepper` shell.
 */

#include "io.h"
#include "roles.h"
#include "motion.h"
#include "safety.h"
#include "solenoid.h"
#include "process.h"
#include "mset.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/shell/shell.h>

static void mset_print(const struct shell *sh);

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "fault : %s", safety_in_fault() ? "LATCHED" : "clear");
	shell_print(sh, "ena   : axis1=%s axis2=%s axis3=%s",
		    safety_enabled(0) ? "ON" : "free",
		    safety_enabled(1) ? "ON" : "free",
		    safety_enabled(2) ? "ON" : "free");
	shell_print(sh, "laser : 1=%d 2=%d 3=%d",
		    io_sensor(IO_LASER1), io_sensor(IO_LASER2), io_sensor(IO_LASER3));
	shell_print(sh, "induct: 4=%d 5=%d 6=%d 7=%d 8=%d 9=%d",
		    io_sensor(IO_INDUCTIVE4), io_sensor(IO_INDUCTIVE5), io_sensor(IO_INDUCTIVE6),
		    io_sensor(IO_INDUCTIVE7), io_sensor(IO_INDUCTIVE8), io_sensor(IO_INDUCTIVE9));
	shell_print(sh, "photo : 10=%d 11=%d",
		    io_sensor(IO_PHOTO10), io_sensor(IO_PHOTO11));
	shell_print(sh, "drive : ALM/PEND 1=%d/%d 2=%d/%d 3=%d/%d 4=%d/%d",
		    io_alm(IO_DRIVE1), io_pend(IO_DRIVE1),
		    io_alm(IO_DRIVE2), io_pend(IO_DRIVE2),
		    io_alm(IO_DRIVE3), io_pend(IO_DRIVE3),
		    io_alm(IO_DRIVE4), io_pend(IO_DRIVE4));
	shell_print(sh, "smema : up_board_avail=%d down_mach_ready=%d",
		    io_smema_upstream_board_available(),
		    io_smema_downstream_machine_ready());

	char line[128];
	size_t off = 0;
	int cnt = 0;

	for (int s = 0; s < IO_SENSOR_COUNT; s++) {
		if (io_sensor(s)) {
			off += snprintf(line + off, sizeof(line) - off, "%s%s",
					cnt ? " " : "", io_sensor_name(s));
			cnt++;
		}
	}
	shell_print(sh, "assert: %s  (check any unexpected at rest -> wrong NO/NC?)",
		    cnt ? line : "(none)");
	if (io_sim_any()) {
		shell_warn(sh, "SIM ACTIVE - inputs forced; `infeed sim off` for live pins");
	}
	mset_print(sh);
	return 0;
}

static int cmd_enable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	int axis = atoi(argv[1]);

	if (axis < 1 || axis > SAFETY_AXES) {
		shell_error(sh, "axis must be 1-%d", SAFETY_AXES);
		return -EINVAL;
	}

	bool on;

	if (strcmp(argv[2], "on") == 0) {
		on = true;
	} else if (strcmp(argv[2], "off") == 0) {
		on = false;
	} else {
		shell_error(sh, "expected on|off");
		return -EINVAL;
	}

	int err = safety_set_enable(axis - 1, on);

	if (err == -EBUSY) {
		shell_error(sh, "refused: fault latched (run `infeed reset` first)");
		return err;
	}
	if (err) {
		shell_error(sh, "enable failed: %d", err);
		return err;
	}
	shell_print(sh, "axis%d %s", axis, on ? "ENABLED" : "free");
	return 0;
}

static int cmd_sol(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	int id = atoi(argv[1]);

	if (id < 1 || id > SOLENOID_COUNT) {
		shell_error(sh, "solenoid must be 1-%d", SOLENOID_COUNT);
		return -EINVAL;
	}

	enum solenoid_pos pos;

	if (strcmp(argv[2], "a") == 0) {
		pos = SOLENOID_A;
	} else if (strcmp(argv[2], "b") == 0) {
		pos = SOLENOID_B;
	} else {
		shell_error(sh, "expected a|b");
		return -EINVAL;
	}

	int err = solenoid_set(id - 1, pos);

	if (err) {
		shell_error(sh, "solenoid failed: %d", err);
		return err;
	}
	shell_print(sh, "sol%d -> coil %s held (level)", id, pos == SOLENOID_A ? "A" : "B");
	return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	motion_stop_all();
	shell_print(sh, "all axes stopped");
	return 0;
}

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!safety_in_fault()) {
		shell_print(sh, "no fault latched");
		return 0;
	}
	safety_fault_reset();
	shell_print(sh, "fault cleared (motors free; re-enable explicitly)");
	return 0;
}

static int cmd_abort(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (safety_in_fault()) {
		shell_print(sh, "already aborted/faulted - run `infeed reset` to restart");
		return 0;
	}
	safety_raise_fault("operator abort (infeed abort)");
	shell_print(sh, "ABORTED - motion stopped, motors free, valves hold.");
	shell_print(sh, "run `infeed reset` to re-home and restart the cycle at idle");
	return 0;
}

/* Teach: bind a role to whichever sensor the installer actuates next. */
static int teach_role(const struct shell *sh, int r)
{
	bool base[IO_SENSOR_COUNT];

	for (int s = 0; s < IO_SENSOR_COUNT; s++) {
		base[s] = io_sensor(s);
	}
	shell_print(sh, "TEACH %s: actuate its sensor now (15 s)...", role_name(r));

	for (int t = 0; t < 1500; t++) {
		k_msleep(10);
		for (int s = 0; s < IO_SENSOR_COUNT; s++) {
			if (io_sensor(s) != base[s]) {
				role_assign(r, (enum io_sensor_id)s);
				shell_print(sh, "%s <- %s (%s)", role_name(r),
					    io_sensor_name(s), io_sensor_pin(s));
				shell_warn(sh, "RAM only - run `infeed cfg save` to persist");
				return 0;
			}
		}
	}
	shell_warn(sh, "timeout - no input changed; nothing bound");
	return 0;
}

static int cmd_map(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		int use[IO_SENSOR_COUNT] = {0};

		shell_print(sh, "  %-18s %-12s %-5s asserted", "role", "sensor", "pin");
		for (int r = 0; r < ROLE_COUNT; r++) {
			enum io_sensor_id s = role_sensor(r);

			shell_print(sh, "  %-18s %-12s %-5s %d",
				    role_name(r), io_sensor_name(s),
				    io_sensor_pin(s), role_read(r));
			use[s]++;
		}
		for (int s = 0; s < IO_SENSOR_COUNT; s++) {
			if (use[s] > 1) {
				shell_warn(sh, "  ! %s bound to >1 role", io_sensor_name(s));
			}
		}
		for (int s = 0; s < IO_SENSOR_COUNT; s++) {
			if (use[s] == 0) {
				shell_print(sh, "  (unused: %s %s)",
					    io_sensor_name(s), io_sensor_pin(s));
			}
		}
		shell_print(sh, "set: infeed map <role> <sensor>  |  teach: infeed map teach <role>");
		return 0;
	}

	if (strcmp(argv[1], "teach") == 0) {
		if (argc != 3) {
			shell_error(sh, "usage: infeed map teach <role>");
			return -EINVAL;
		}
		int r = role_by_name(argv[2]);

		if (r < 0) {
			shell_error(sh, "unknown role '%s'", argv[2]);
			return -EINVAL;
		}
		return teach_role(sh, r);
	}

	if (argc != 3) {
		shell_error(sh, "usage: infeed map [<role> <sensor>] | teach <role>");
		return -EINVAL;
	}

	int r = role_by_name(argv[1]);
	int s = sensor_by_name(argv[2]);

	if (r < 0) {
		shell_error(sh, "unknown role '%s' (try `infeed map`)", argv[1]);
		return -EINVAL;
	}
	if (s < 0) {
		shell_error(sh, "unknown sensor '%s'", argv[2]);
		return -EINVAL;
	}
	role_assign(r, s);
	shell_print(sh, "%s <- %s (%s)", role_name(r),
		    io_sensor_name(s), io_sensor_pin(s));
	shell_warn(sh, "RAM only - run `infeed cfg save` to persist");
	return 0;
}

static int cmd_watch(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t secs = (argc > 1) ? (uint32_t)atoi(argv[1]) : 20;
	bool prev[IO_SENSOR_COUNT];

	if (secs < 1) {
		secs = 1;
	}
	if (secs > 120) {
		secs = 120;
	}
	shell_print(sh, "watch %u s - '*' = asserted now (verify expected at rest):", secs);
	for (int s = 0; s < IO_SENSOR_COUNT; s++) {
		prev[s] = io_sensor(s);
		shell_print(sh, "  %-12s %-5s = %d %s", io_sensor_name(s),
			    io_sensor_pin(s), prev[s], prev[s] ? "*" : "");
	}
	for (uint32_t t = 0; t < secs * 10; t++) {
		k_msleep(100);
		for (int s = 0; s < IO_SENSOR_COUNT; s++) {
			bool v = io_sensor(s);

			if (v != prev[s]) {
				shell_print(sh, "  %-12s %-5s -> %d", io_sensor_name(s),
					    io_sensor_pin(s), v);
				prev[s] = v;
			}
		}
	}
	shell_print(sh, "watch done");
	return 0;
}

static int cmd_dir(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		for (int a = 0; a < MOTION_AXIS_COUNT; a++) {
			shell_print(sh, "  bank%d : %s", a + 1,
				    motion_invert(a) ? "INVERTED" : "normal");
		}
		shell_print(sh, "set: infeed dir <1-3> <norm|inv>");
		return 0;
	}
	if (argc != 3) {
		shell_error(sh, "usage: infeed dir [<1-3> <norm|inv>]");
		return -EINVAL;
	}

	int b = atoi(argv[1]);

	if (b < 1 || b > MOTION_AXIS_COUNT) {
		shell_error(sh, "bank must be 1-%d", MOTION_AXIS_COUNT);
		return -EINVAL;
	}

	bool inv;

	if (strcmp(argv[2], "inv") == 0) {
		inv = true;
	} else if (strcmp(argv[2], "norm") == 0) {
		inv = false;
	} else {
		shell_error(sh, "expected norm|inv");
		return -EINVAL;
	}
	motion_set_invert(b - 1, inv);
	shell_print(sh, "bank%d = %s", b, inv ? "INVERTED" : "normal");
	shell_warn(sh, "RAM only - run `infeed cfg save` to persist");
	return 0;
}

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* `infeed start` (re)triggers a fresh run: stop any motion, re-home, and
	 * (re)start the per-panel cycle. Works any time, no reboot. */
	process_start();
	shell_print(sh, "start: re-homing and (re)starting the cycle");
	return 0;
}

/* Report a manual portion-of-cycle command result. */
static int manual_report(const struct shell *sh, int err, const char *ok)
{
	switch (err) {
	case 0:
		shell_print(sh, "%s (manual mode - run `infeed start` to resume the auto-cycle)", ok);
		return 0;
	case -EBUSY:
		shell_error(sh, "fault latched - run `infeed reset` first");
		return err;
	case -EAGAIN:
		shell_error(sh, "auto cycle running - `infeed abort` then retry");
		return err;
	default:
		shell_error(sh, "interrupted (fault or `infeed start`)");
		return err;
	}
}

static int cmd_table_home(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return manual_report(sh, process_table_home(), "table homed");
}

static int cmd_table_down(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return manual_report(sh, process_table_down(), "table down to test position");
}

static int cmd_load(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return manual_report(sh, process_load(), "loaded: panel conveyed to position");
}

static int cmd_unload(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return manual_report(sh, process_unload(), "unloaded: panel conveyed out past Laser 3");
}

static int cmd_test(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_print(sh, "test mode %s - cylinder confirms %s",
			    process_test_mode() ? "ON" : "off",
			    process_test_mode() ? "wait forever" : "alarm after 10 s");
		return 0;
	}
	if (strcmp(argv[1], "on") == 0) {
		process_set_test_mode(true);
		shell_print(sh, "test mode ON - cylinder confirms wait indefinitely (hand-stepping)");
		return 0;
	}
	if (strcmp(argv[1], "off") == 0) {
		process_set_test_mode(false);
		shell_print(sh, "test mode off - cylinder alarm after 10 s (production)");
		return 0;
	}
	shell_error(sh, "expected on|off");
	return -EINVAL;
}

/* Logic test: force an input on/off so the cycle can be walked without sensors. */
static int cmd_sim(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_print(sh, "simulation %s.  usage: infeed sim <name> <0|1> | off",
			    io_sim_any() ? "ACTIVE" : "off");
		shell_print(sh, "names: laser1..3  inductive4..9  photo10 photo11  smema_up  smema_down");
		return 0;
	}
	if (strcmp(argv[1], "off") == 0) {
		io_sim_off();
		shell_print(sh, "simulation off - live inputs");
		return 0;
	}
	if (argc != 3) {
		shell_error(sh, "usage: infeed sim <name> <0|1> | off");
		return -EINVAL;
	}

	bool on = (argv[2][0] == '1');
	int s = sensor_by_name(argv[1]);

	if (s >= 0) {
		io_sim_sensor((enum io_sensor_id)s, on);
	} else if (strcmp(argv[1], "smema_up") == 0) {
		io_sim_smema_up_board_available(on);
	} else if (strcmp(argv[1], "smema_down") == 0) {
		io_sim_smema_down_machine_ready(on);
	} else {
		shell_error(sh, "unknown input '%s' (try `infeed sim`)", argv[1]);
		return -EINVAL;
	}
	shell_print(sh, "sim %s = %d", argv[1], on);
	return 0;
}

/* Manual jog: `infeed jog <1-3> <+|-> [rev=1] [rpm=20]` (bench). */
static int cmd_jog(const struct shell *sh, size_t argc, char **argv)
{
	int b = atoi(argv[1]);

	if (b < 1 || b > MOTION_AXIS_COUNT) {
		shell_error(sh, "bank must be 1-%d", MOTION_AXIS_COUNT);
		return -EINVAL;
	}

	enum motion_dir dir;

	if (strcmp(argv[2], "+") == 0) {
		dir = MOTION_POS;
	} else if (strcmp(argv[2], "-") == 0) {
		dir = MOTION_NEG;
	} else {
		shell_error(sh, "direction must be + or -");
		return -EINVAL;
	}

	double rev = (argc > 3) ? strtod(argv[3], NULL) : 1.0;
	uint32_t rpm = (argc > 4) ? (uint32_t)atoi(argv[4]) : 20;

	if (!(rev > 0.0) || rev > 50.0) {
		shell_error(sh, "rev must be > 0 and <= 50");
		return -EINVAL;
	}
	if (rpm < 1 || rpm > 300) {
		shell_error(sh, "rpm must be 1-300");
		return -EINVAL;
	}
	if (safety_in_fault()) {
		shell_error(sh, "fault latched - run `infeed reset` first");
		return -EBUSY;
	}
	if (motion_is_moving(b - 1)) {
		shell_error(sh, "bank%d is moving - `infeed stop` first", b);
		return -EBUSY;
	}

	uint32_t steps = (uint32_t)(rev * (double)MOTION_STEPS_PER_REV + 0.5);

	(void)safety_set_enable(b - 1, true);   /* a jog needs holding torque */

	int err = motion_go_steps(b - 1, dir, steps, rpm);

	if (err) {
		shell_error(sh, "jog failed: %d", err);
		return err;
	}
	shell_print(sh, "bank%d jog %s%s rev @ %u rpm (%u steps), bank enabled",
		    b, argv[2], (argc > 3) ? argv[3] : "1", rpm, steps);
	return 0;
}

static int cmd_pol(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		for (int s = 0; s < IO_SENSOR_COUNT; s++) {
			shell_print(sh, "  %-12s %-5s %s",
				    io_sensor_name(s), io_sensor_pin(s),
				    io_active_high(s) ? "active-HIGH (NO)"
						      : "active-LOW (NC)");
		}
		shell_print(sh, "set: infeed pol <sensor> <high|low>");
		return 0;
	}
	if (argc != 3) {
		shell_error(sh, "usage: infeed pol [<sensor> <high|low>]");
		return -EINVAL;
	}

	int s = sensor_by_name(argv[1]);

	if (s < 0) {
		shell_error(sh, "unknown sensor '%s'", argv[1]);
		return -EINVAL;
	}

	bool ah;

	if (strcmp(argv[2], "high") == 0) {
		ah = true;
	} else if (strcmp(argv[2], "low") == 0) {
		ah = false;
	} else {
		shell_error(sh, "expected high|low");
		return -EINVAL;
	}
	io_set_active_high(s, ah);
	io_refresh();
	shell_print(sh, "%s = active-%s", io_sensor_name(s), ah ? "HIGH" : "LOW");
	shell_warn(sh, "RAM only - run `infeed cfg save` to persist");
	return 0;
}

/* Accept both '.' and ',' as the decimal separator (operator convenience). */
static double parse_decimal(const char *s)
{
	char buf[24];
	size_t i = 0;

	for (; s[i] && i < sizeof(buf) - 1; i++) {
		buf[i] = (s[i] == ',') ? '.' : s[i];
	}
	buf[i] = '\0';
	return strtod(buf, NULL);
}

static void mset_print(const struct shell *sh)
{
	shell_print(sh, "motor setup (infeed set <1-7> <rpm> [rev]):");
	for (int id = 1; id <= MSET_COUNT; id++) {
		if (mset_has_rounds((enum mset_id)id)) {
			uint32_t mrev = mset_steps((enum mset_id)id) * 1000U / MOTION_STEPS_PER_REV;

			shell_print(sh, "  set %d: %u rpm, %u.%03u rev   [%s]", id,
				    mset_rpm((enum mset_id)id), mrev / 1000U, mrev % 1000U,
				    mset_name((enum mset_id)id));
		} else {
			shell_print(sh, "  set %d: %u rpm             [%s]", id,
				    mset_rpm((enum mset_id)id), mset_name((enum mset_id)id));
		}
	}
}

static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		mset_print(sh);
		return 0;
	}

	int id = atoi(argv[1]);

	if (id < 1 || id > MSET_COUNT) {
		shell_error(sh, "id must be 1-%d (see `infeed set`)", MSET_COUNT);
		return -EINVAL;
	}
	if (argc < 3) {
		shell_error(sh, "usage: infeed set %d <rpm>%s", id,
			    mset_has_rounds((enum mset_id)id) ? " <rev>" : "");
		return -EINVAL;
	}

	int rpm = atoi(argv[2]);

	if (rpm <= 0) {
		shell_error(sh, "rpm must be > 0");
		return -EINVAL;
	}

	uint32_t steps = 0;

	if (mset_has_rounds((enum mset_id)id)) {
		if (argc < 4) {
			shell_error(sh, "set %d needs: <rpm> <rev>", id);
			return -EINVAL;
		}
		double rev = parse_decimal(argv[3]);

		if (!(rev > 0.0)) {
			shell_error(sh, "rev must be > 0 (use '.' or ',')");
			return -EINVAL;
		}
		steps = (uint32_t)(rev * (double)MOTION_STEPS_PER_REV + 0.5);
	}

	(void)mset_set((enum mset_id)id, (uint32_t)rpm, steps);

	if (mset_has_rounds((enum mset_id)id)) {
		uint32_t mrev = steps * 1000U / MOTION_STEPS_PER_REV;

		shell_print(sh, "set %d = %d rpm, %u.%03u rev  [%s]  (run `infeed cfg save`)",
			    id, rpm, mrev / 1000U, mrev % 1000U, mset_name((enum mset_id)id));
	} else {
		shell_print(sh, "set %d = %d rpm  [%s]  (run `infeed cfg save`)",
			    id, rpm, mset_name((enum mset_id)id));
	}
	return 0;
}

static int cmd_cfg(const struct shell *sh, size_t argc, char **argv)
{
	if (strcmp(argv[1], "save") == 0) {
		int rc = roles_save();

		if (rc == 0) {
			rc = mset_save();
		}
		if (rc) {
			shell_error(sh, "save failed: %d", rc);
			return rc;
		}
		shell_print(sh, "saved to flash (roles + motor setup)");
		return 0;
	}
	if (strcmp(argv[1], "reset") == 0) {
		int rc = roles_reset();

		if (rc) {
			shell_error(sh, "reset failed: %d", rc);
			return rc;
		}
		shell_print(sh, "defaults restored + saved");
		return 0;
	}
	if (strcmp(argv[1], "dump") == 0) {
		char tok[48];
		int rc = roles_dump(tok, sizeof(tok));

		if (rc) {
			shell_error(sh, "dump failed: %d", rc);
			return rc;
		}
		shell_print(sh, "%s", tok);
		return 0;
	}
	if (strcmp(argv[1], "load") == 0) {
		if (argc != 3) {
			shell_error(sh, "usage: infeed cfg load <token>");
			return -EINVAL;
		}
		int rc = roles_apply_token(argv[2]);

		if (rc) {
			shell_error(sh, "bad token: %d", rc);
			return rc;
		}
		shell_print(sh, "loaded");
		shell_warn(sh, "RAM only - run `infeed cfg save` to persist");
		return 0;
	}
	shell_error(sh, "expected save|reset|dump|load");
	return -EINVAL;
}

/* `infeed table home|down` (manual commissioning). */
SHELL_STATIC_SUBCMD_SET_CREATE(table_cmds,
	SHELL_CMD(home, NULL, "home the table up to Photo 10 (#1)", cmd_table_home),
	SHELL_CMD(down, NULL, "table down to test position, 6 rev (#10)", cmd_table_down),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(infeed_cmds,
	SHELL_CMD(status, NULL,
		  "Dump sensors / ALM / PEND / SMEMA / fault / enable state",
		  cmd_status),
	SHELL_CMD_ARG(enable, NULL, "<1-3> <on|off>  enable or free a bank",
		      cmd_enable, 3, 0),
	SHELL_CMD_ARG(sol, NULL, "<1-3> <a|b>  pulse a valve solenoid",
		      cmd_sol, 3, 0),
	SHELL_CMD(stop, NULL, "Stop all axes", cmd_stop),
	SHELL_CMD(abort, NULL, "Software E-stop: abort the cycle (reset to restart)", cmd_abort),
	SHELL_CMD(reset, NULL, "Clear a latched fault / abort (re-homes, restarts)", cmd_reset),
	SHELL_CMD(start, NULL, "Release the unconfigured safe-hold / begin", cmd_start),
	SHELL_CMD(table, &table_cmds, "table home|down  (manual commissioning)", NULL),
	SHELL_CMD(load, NULL, "manual: panel in (Laser1) -> convey -> creep (#4-#9)", cmd_load),
	SHELL_CMD(unload, NULL, "manual: convey out to position (Laser3) -> stop (#25-#27)", cmd_unload),
	SHELL_CMD_ARG(test, NULL, "[on|off]  commissioning mode: cylinder confirms wait (no alarm)",
		      cmd_test, 1, 1),
	SHELL_CMD_ARG(map, NULL,
		      "[<role> <sensor>] | teach <role>  show/set/teach the sensor-role map",
		      cmd_map, 1, 2),
	SHELL_CMD_ARG(watch, NULL, "[secs]  live-watch the sensor inputs", cmd_watch, 1, 1),
	SHELL_CMD_ARG(pol, NULL, "[<sensor> <high|low>]  show/set sensor active level",
		      cmd_pol, 1, 2),
	SHELL_CMD_ARG(dir, NULL, "[<1-3> <norm|inv>]  show/set per-bank direction",
		      cmd_dir, 1, 2),
	SHELL_CMD_ARG(jog, NULL, "<1-3> <+|-> [rev=1] [rpm=20]  manually jog a bank",
		      cmd_jog, 3, 2),
	SHELL_CMD_ARG(sim, NULL, "<name> <0|1> | off  force an input (logic test)",
		      cmd_sim, 1, 2),
	SHELL_CMD_ARG(set, NULL, "[<1-7> <rpm> [rev]]  motor speeds/rounds (no args = show)",
		      cmd_set, 1, 3),
	SHELL_CMD_ARG(cfg, NULL, "<save|reset|dump|load <token>>  persistence",
		      cmd_cfg, 2, 1),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(infeed, &infeed_cmds, "INFEED controller commands", NULL);
