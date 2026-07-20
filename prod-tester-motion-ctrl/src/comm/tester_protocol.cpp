#include "tester_protocol.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tester_proto, LOG_LEVEL_INF);

namespace comm {

int TesterProtocol::init()
{
    LOG_INF("tester protocol init (USART3 stub)");
    return 0;
}

void TesterProtocol::poll()
{
    // TODO: read from USART3, parse line, dispatch callback
}

void TesterProtocol::sendEvent(TesterEvent evt, const char *data)
{
    // TODO: format and send over USART3
    (void)evt;
    (void)data;
}

} // namespace comm
