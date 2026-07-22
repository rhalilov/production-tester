#include "app.h"

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

namespace app {

static engine::Engine *s_engine;
static engine::Context *s_ctx;
static struct k_timer *s_timer;
static AccessLevel s_access = AccessLevel::OPERATOR;

static void update_prompt()
{
    const struct shell *sh = shell_backend_uart_get_ptr();
    if (!sh) return;

    switch (s_access) {
    case AccessLevel::OPERATOR:
        shell_prompt_change(sh, "\x1b[32moperator\x1b[0m> ");
        break;
    case AccessLevel::PROC_ENG:
        shell_prompt_change(sh, "\x1b[33mproc_eng\x1b[0m> ");
        break;
    case AccessLevel::FACTORY:
        shell_prompt_change(sh, "\x1b[1;31mFACTORY\x1b[0m> ");
        break;
    }
}

void init(engine::Engine *eng, engine::Context *ctx, struct k_timer *timer)
{
    s_engine = eng;
    s_ctx = ctx;
    s_timer = timer;
    update_prompt();
}

engine::Engine *engine() { return s_engine; }
engine::Context *context() { return s_ctx; }

AccessLevel accessLevel() { return s_access; }
void setAccessLevel(AccessLevel level)
{
    s_access = level;
    update_prompt();
}
bool isFactory() { return s_access == AccessLevel::FACTORY; }

void startScan()
{
    if (s_timer) {
        k_timer_start(s_timer, K_MSEC(CONFIG_SCAN_PERIOD_MS),
                      K_MSEC(CONFIG_SCAN_PERIOD_MS));
        LOG_INF("Scan timer started (%d ms)", CONFIG_SCAN_PERIOD_MS);
    }
}

void stopScan()
{
    if (s_timer) {
        k_timer_stop(s_timer);
        LOG_INF("Scan timer stopped");
    }
}

} // namespace app
