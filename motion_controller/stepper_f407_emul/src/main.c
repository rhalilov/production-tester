/*
 * INFEED multi-axis stepper controller — entry point.
 *
 * Thin by design: just bring the modules up in order and hand off to the
 * supervisor thread (started by safety_init). All behaviour lives in the
 * modules: io (pins/EXTI), motion (3 axes), home, smema, safety (supervisor).
 */

#include "io.h"
#include "roles.h"
#include "motion.h"
#include "home.h"
#include "smema.h"
#include "solenoid.h"
#include "safety.h"
#include "process.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Keep the SWD/debug interface alive during WFI sleep so Windows ST-Link /
 * CubeProgrammer can ALWAYS connect. On STM32F4 the debug link is dropped in
 * sleep by default -> a normal/hot-plug connect fails with "No device
 * connected" (only connect-under-reset works). Setting DBGMCU_CR
 * DBG_SLEEP|DBG_STOP|DBG_STANDBY keeps the debug clock running so the board
 * never blocks programming. */
#define DBGMCU_CR 0xE0042004u
static int keep_debug_alive(void)
{
	sys_write32(sys_read32(DBGMCU_CR) | 0x7u, DBGMCU_CR);
	return 0;
}
SYS_INIT(keep_debug_alive, PRE_KERNEL_1, 0);

/* Second interactive shell on USART3 (PD8/PD9), mirroring the USB-CDC console.
 * Both shells accept `infeed` commands independently; LOG + the evlog cycle
 * narration fan to BOTH (CONFIG_LOG_PRINTK routes printk through the log
 * backends, and this shell registers its own log backend). Replicates the
 * built-in shell_uart instantiation, bound to USART3 instead of the chosen. */
SHELL_UART_DEFINE(shell_transport_uart3);
SHELL_DEFINE(shell_uart3, "infeed:~$ ", &shell_transport_uart3,
	     CONFIG_SHELL_BACKEND_SERIAL_LOG_MESSAGE_QUEUE_SIZE,
	     CONFIG_SHELL_BACKEND_SERIAL_LOG_MESSAGE_QUEUE_TIMEOUT,
	     SHELL_FLAG_OLF_CRLF);

static int shell_uart3_init(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(usart3));
	static const struct shell_backend_config_flags cfg = SHELL_DEFAULT_BACKEND_CONFIG_FLAGS;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	/* log_backend = true -> LOG + evlog also render on PD8/PD9 */
	return shell_init(&shell_uart3, dev, cfg, true, CONFIG_LOG_MAX_LEVEL);
}
SYS_INIT(shell_uart3_init, POST_KERNEL, CONFIG_SHELL_BACKEND_SERIAL_INIT_PRIORITY);

int main(void)
{
	int err;

	LOG_INF("INFEED multi-axis controller starting");

	err = io_init();
	if (err) {
		LOG_ERR("io_init: %d", err);
	}
	err = roles_init();   /* sensor-role map + polarity (loads persisted config) */
	if (err) {
		LOG_ERR("roles_init: %d", err);
	}
	err = motion_init();
	if (err) {
		LOG_ERR("motion_init: %d", err);
	}
	err = home_init();
	if (err) {
		LOG_ERR("home_init: %d", err);
	}
	err = smema_init();
	if (err) {
		LOG_ERR("smema_init: %d", err);
	}
	err = solenoid_init();
	if (err) {
		LOG_ERR("solenoid_init: %d", err);
	}

	/* Supervisor thread (ALM/fault, LEDs, watchdog) — after all I/O is set. */
	err = safety_init();
	if (err) {
		LOG_ERR("safety_init: %d", err);
	}

	/* Last: the operation sequence (homing + per-panel cycle). */
	err = process_init();
	if (err) {
		LOG_ERR("process_init: %d", err);
	}

	LOG_INF("init complete");
	return 0;
}
