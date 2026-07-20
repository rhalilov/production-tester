#include "smema.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(smema, LOG_LEVEL_INF);

int smema_init(void)
{
	/* TODO: ready/board-available handshake state machine via io_smema_*(). */
	LOG_INF("SMEMA reserved (pins owned by io)");
	return 0;
}
