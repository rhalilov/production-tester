#include "roles.h"
#include "io.h"
#include "motion.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(roles, LOG_LEVEL_INF);

/* Compiled default binding (current wiring). */
static const uint8_t map_default[ROLE_COUNT] = {
	[ROLE_PANEL_PRESENTED]   = IO_LASER1,
	[ROLE_PANEL_NEAR]        = IO_LASER2,
	[ROLE_PANEL_IN_POSITION] = IO_LASER3,
	[ROLE_TABLE_HOME]        = IO_PHOTO10,
	[ROLE_CONVEYOR_WIDTH]    = IO_PHOTO11,
	[ROLE_CYL1_DOWN]         = IO_INDUCTIVE4,
	[ROLE_CYL1_UP]           = IO_INDUCTIVE5,
	[ROLE_CYL2_DOWN]         = IO_INDUCTIVE6,
	[ROLE_CYL2_UP]           = IO_INDUCTIVE7,
	[ROLE_CYL3_DOWN]         = IO_INDUCTIVE8,
	[ROLE_CYL3_UP]           = IO_INDUCTIVE9,
};

static uint8_t map[ROLE_COUNT];
static bool    configured;   /* a saved config was loaded from flash */

static const char *const role_names[ROLE_COUNT] = {
	"panel_presented", "panel_near", "panel_in_position", "table_home",
	"conveyor_width",
	"cyl1_down", "cyl1_up", "cyl2_down", "cyl2_up", "cyl3_down", "cyl3_up",
};

bool role_read(enum io_role r)
{
	return io_sensor((enum io_sensor_id)map[r]);
}

enum io_sensor_id role_sensor(enum io_role r)
{
	return (enum io_sensor_id)map[r];
}

int role_assign(enum io_role r, enum io_sensor_id s)
{
	if ((int)r < 0 || r >= ROLE_COUNT || (int)s < 0 || s >= IO_SENSOR_COUNT) {
		return -EINVAL;
	}
	map[r] = (uint8_t)s;
	return 0;
}

const char *role_name(enum io_role r)
{
	return ((int)r < ROLE_COUNT) ? role_names[r] : "?";
}

int role_by_name(const char *name)
{
	for (int r = 0; r < ROLE_COUNT; r++) {
		if (strcmp(role_names[r], name) == 0) {
			return r;
		}
	}
	return -1;
}

int sensor_by_name(const char *name)
{
	for (int s = 0; s < IO_SENSOR_COUNT; s++) {
		if (strcmp(io_sensor_name((enum io_sensor_id)s), name) == 0) {
			return s;
		}
	}
	return -1;
}

bool roles_is_configured(void)
{
	return configured;
}

/* ---- portable config token: "INF1:" + hex(map[11] pol[2] dir[1] cks[1]) -- */
#define TOKEN_BYTES (ROLE_COUNT + 2 + 1)   /* map + pol(u16) + dir = 14 */

int roles_dump(char *out, size_t n)
{
	uint8_t buf[TOKEN_BYTES];
	uint16_t pol = 0;
	uint8_t dir = 0, cks = 0;
	int k = 0;
	size_t off;

	for (int r = 0; r < ROLE_COUNT; r++) {
		buf[k++] = map[r];
	}
	for (int s = 0; s < IO_SENSOR_COUNT; s++) {
		if (io_active_high((enum io_sensor_id)s)) {
			pol |= BIT(s);
		}
	}
	buf[k++] = (uint8_t)(pol & 0xff);
	buf[k++] = (uint8_t)(pol >> 8);
	for (int a = 0; a < MOTION_AXIS_COUNT; a++) {
		if (motion_invert((enum motion_axis)a)) {
			dir |= BIT(a);
		}
	}
	buf[k++] = dir;
	for (int i = 0; i < TOKEN_BYTES; i++) {
		cks += buf[i];
	}

	if (n < 5 + (TOKEN_BYTES + 1) * 2 + 1) {
		return -ENOMEM;
	}
	off = (size_t)snprintf(out, n, "INF1:");
	for (int i = 0; i < TOKEN_BYTES; i++) {
		off += (size_t)snprintf(out + off, n - off, "%02x", buf[i]);
	}
	(void)snprintf(out + off, n - off, "%02x", cks);
	return 0;
}

static int hexval(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}
	return -1;
}

int roles_apply_token(const char *tok)
{
	uint8_t buf[TOKEN_BYTES + 1];
	uint8_t cks = 0;

	if (strncmp(tok, "INF1:", 5) != 0) {
		return -EINVAL;
	}
	tok += 5;
	if (strlen(tok) != (TOKEN_BYTES + 1) * 2) {
		return -EINVAL;
	}
	for (int i = 0; i < TOKEN_BYTES + 1; i++) {
		int hi = hexval(tok[2 * i]);
		int lo = hexval(tok[2 * i + 1]);

		if (hi < 0 || lo < 0) {
			return -EINVAL;
		}
		buf[i] = (uint8_t)((hi << 4) | lo);
	}
	for (int i = 0; i < TOKEN_BYTES; i++) {
		cks += buf[i];
	}
	if (cks != buf[TOKEN_BYTES]) {
		return -EBADMSG;
	}
	for (int r = 0; r < ROLE_COUNT; r++) {
		if (buf[r] >= IO_SENSOR_COUNT) {
			return -EINVAL;
		}
	}

	for (int r = 0; r < ROLE_COUNT; r++) {
		map[r] = buf[r];
	}
	uint16_t pol = (uint16_t)buf[ROLE_COUNT] | ((uint16_t)buf[ROLE_COUNT + 1] << 8);

	for (int s = 0; s < IO_SENSOR_COUNT; s++) {
		io_set_active_high((enum io_sensor_id)s, pol & BIT(s));
	}
	for (int a = 0; a < MOTION_AXIS_COUNT; a++) {
		motion_set_invert((enum motion_axis)a, buf[TOKEN_BYTES - 1] & BIT(a));
	}
	io_refresh();
	return 0;
}

/* ---- persistence (NVS-backed settings) -------------------------------- */
/* Keys under "infeed": map (ids), pol (active-high u16), dir (invert bits). */

static int roles_set(const char *name, size_t len, settings_read_cb read_cb,
		     void *cb_arg)
{
	const char *next;

	if (settings_name_steq(name, "map", &next) && !next) {
		uint8_t b[ROLE_COUNT];

		if (len != sizeof(b) || read_cb(cb_arg, b, sizeof(b)) < 0) {
			return -EINVAL;
		}
		for (int r = 0; r < ROLE_COUNT; r++) {
			if (b[r] < IO_SENSOR_COUNT) {
				map[r] = b[r];
			}
		}
		configured = true;
		return 0;
	}
	if (settings_name_steq(name, "pol", &next) && !next) {
		uint16_t bits;

		if (len != sizeof(bits) || read_cb(cb_arg, &bits, sizeof(bits)) < 0) {
			return -EINVAL;
		}
		for (int s = 0; s < IO_SENSOR_COUNT; s++) {
			io_set_active_high((enum io_sensor_id)s, bits & BIT(s));
		}
		return 0;
	}
	if (settings_name_steq(name, "dir", &next) && !next) {
		uint8_t d;

		if (len != sizeof(d) || read_cb(cb_arg, &d, sizeof(d)) < 0) {
			return -EINVAL;
		}
		for (int a = 0; a < MOTION_AXIS_COUNT; a++) {
			motion_set_invert((enum motion_axis)a, d & BIT(a));
		}
		return 0;
	}
	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(infeed_roles, "infeed", NULL, roles_set, NULL, NULL);

int roles_save(void)
{
	uint16_t pol = 0;
	uint8_t dir = 0;
	int rc;

	for (int s = 0; s < IO_SENSOR_COUNT; s++) {
		if (io_active_high((enum io_sensor_id)s)) {
			pol |= BIT(s);
		}
	}
	for (int a = 0; a < MOTION_AXIS_COUNT; a++) {
		if (motion_invert((enum motion_axis)a)) {
			dir |= BIT(a);
		}
	}
	rc = settings_save_one("infeed/map", map, sizeof(map));
	if (rc == 0) {
		rc = settings_save_one("infeed/pol", &pol, sizeof(pol));
	}
	if (rc == 0) {
		rc = settings_save_one("infeed/dir", &dir, sizeof(dir));
	}
	if (rc) {
		LOG_ERR("save: %d", rc);
	} else {
		configured = true;
	}
	return rc;
}

int roles_reset(void)
{
	memcpy(map, map_default, sizeof(map));
	for (int s = 0; s < IO_SENSOR_COUNT; s++) {
		io_set_active_high((enum io_sensor_id)s, true);
	}
	for (int a = 0; a < MOTION_AXIS_COUNT; a++) {
		motion_set_invert((enum motion_axis)a, false);
	}
	io_refresh();
	return roles_save();
}

int roles_init(void)
{
	int rc;

	memcpy(map, map_default, sizeof(map));   /* defaults first */

	rc = settings_subsys_init();
	if (rc) {
		LOG_ERR("settings init: %d (defaults only)", rc);
		return rc;
	}
	rc = settings_load();                    /* roles_set() applies overrides */
	if (rc) {
		LOG_WRN("settings load: %d (defaults only)", rc);
	}
	io_refresh();
	LOG_INF("roles ready (%s)", configured ? "loaded saved config" : "defaults");
	return 0;
}
