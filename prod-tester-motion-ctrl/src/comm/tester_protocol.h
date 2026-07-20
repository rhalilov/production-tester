// Tester communication protocol (USART3)
// Stub — will be filled with full command parsing

#pragma once

namespace comm {

enum class TesterCmd {
    TEST_START,
    TEST_RESULT_PASS,
    TEST_RESULT_FAIL,
    TEST_RESULT_RETRY,
    STEP_ADVANCE,
    CYCLE_ABORT,
};

enum class TesterEvent {
    BOARD_READY,
    CYCLE_DONE_PASS,
    CYCLE_DONE_FAIL,
    FAULT,
    STEP_CHANGED,
    MODE_CHANGED,
};

class TesterProtocol {
public:
    int init();
    void poll();    // check for incoming commands

    void sendEvent(TesterEvent evt, const char *data = nullptr);

    using CmdCallback = void(*)(TesterCmd cmd, void *ctx);
    void setCallback(CmdCallback cb, void *ctx) { cb_ = cb; ctx_ = ctx; }

private:
    CmdCallback cb_ = nullptr;
    void *ctx_ = nullptr;
};

} // namespace comm
