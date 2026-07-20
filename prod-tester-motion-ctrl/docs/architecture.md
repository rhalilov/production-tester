# Architecture — prod-tester-motion-ctrl

## Design Decisions

- **Language:** C++ in "C with classes" style (`-fno-exceptions`, `-fno-rtti`, no STL)
- **Zephyr:** `CONFIG_CPP=y`, `CONFIG_STD_CPP17=y`, `extern "C"` wrappers for Zephyr callbacks
- **MCU:** STM32F407 (STM32F4 Discovery board)
- **RTOS:** Zephyr v4.x
- **Sequence model:** Compile-time (hardcoded C++ per machine), SFC scan-based engine
- **Settings:** Hybrid persistence (factory blob + recipe blobs + runtime keys via ZMS)

---

## Layered Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  User Interface                                              │
│  CLI (Zephyr Shell) + Event Logger                           │
│  USB-CDC Console (operator) │ USART3 (tester communication)  │
├─────────────────────────────────────────────────────────────┤
│  Machine Logic Layer (SFC Engine + per-machine sequence)      │
│  - Scan-based cyclic execution (5-10ms)                      │
│  - Steps with entry/active/exit actions + transitions        │
│  - Sequence triggers: Laser1, Laser2, Laser3                 │
│  - Operating modes: AUTO, STEP, RETRY, REPETITIVE            │
│  - Retry/recovery procedures per checkpoint                  │
├─────────────────────────────────────────────────────────────┤
│  Component Layer                                             │
│  ┌───────────────┐ ┌──────────────┐ ┌───────────────┐       │
│  │ StepperMotor   │ │ Cylinder     │ │ SmemaPort     │       │
│  │ - position     │ │ - goTo(A/B)  │ │ - IPC-9851    │       │
│  │ - soft limits  │ │ - confirm    │ │ - OK/NG route │       │
│  │ - ALM/PEND     │ │ - timeout    │ │ - BA/MR       │       │
│  │ - homing       │ │ - PULSE/LEVEL│ │               │       │
│  └───────────────┘ └──────────────┘ └───────────────┘       │
├─────────────────────────────────────────────────────────────┤
│  HAL / IO                                                    │
│  GPIO Pins, Zephyr Stepper Driver, ZMS/NVS Settings          │
└─────────────────────────────────────────────────────────────┘
```

---

## Component Layer

### StepperMotor

Self-protecting component. Does not make process-level decisions but enforces its own
limits and reacts to alarms.

#### IO Signals

| Signal | Direction | Description |
|--------|-----------|-------------|
| STEP | MCU → Driver | Step pulse |
| DIR | MCU → Driver | Direction |
| ENA | MCU → Driver | Enable (active low, HBS57) |
| ALM | Driver → MCU | Alarm (fault or force threshold) |
| PEND | Driver → MCU | Positioning complete (optional, not all axes) |

#### Configuration (per-instance)

```cpp
struct StepperMotorConfig {
    // Outputs
    const gpio_dt_spec *step_pin;
    const gpio_dt_spec *dir_pin;
    const gpio_dt_spec *ena_pin;

    // Inputs — multiple for dual-driver axes (table)
    const gpio_dt_spec *alm_pins;
    uint8_t alm_count;              // 1 for conveyor/width, 2 for table
    AlmPolicy alm_policy;           // ANY_TRIGGERS_FAULT

    const gpio_dt_spec *pend_pins;
    uint8_t pend_count;             // 0 for conveyor/width, 2 for table
    PendPolicy pend_policy;         // ALL_REQUIRED

    // Homing (optional)
    bool has_home;
    const gpio_dt_spec *home_sensor;
    MotionDir home_dir;
    bool auto_home;                 // false for width (dangerous at boot)
    uint32_t home_rpm;

    // Soft limits (optional, positional axes only)
    bool has_limits;
    int32_t soft_limit_min;         // steps from home
    int32_t soft_limit_max;
    int32_t safe_position;          // go-to on alarm recovery

    // Position persistence
    bool persist_position;          // save position on shutdown/fault
};
```

#### Instances (this machine)

| Axis | Motors | ALM | PEND | Home | Soft limits | Notes |
|------|--------|-----|------|------|-------------|-------|
| Axis1 (conveyor) | 1 | 1 | none | none | none | Continuous, bidirectional, stops on command from machine logic |
| Axis2 (width) | 1 | 1 | none | yes | yes | auto_home = false (board may be on conveyor) |
| Axis3 (table/needles) | 2 (shared STEP/DIR/ENA) | 2 | 2 | yes (Photo10) | yes | auto_home = true at boot |

#### API

```cpp
class StepperMotor {
public:
    int init();
    int home();                                    // one-time, if has_home
    int go(int32_t steps, uint32_t rpm);           // blocking, waits PEND if available
    int goTo(int32_t position, uint32_t rpm);      // absolute position (from home)
    int run(uint32_t rpm, MotionDir dir);          // continuous
    void stop();
    void emergencyStop();                          // immediate, no decel

    // State
    bool isMoving();
    int32_t position();                            // steps from home
    bool isHomed();
    bool inAlarm();

    // Callback — machine logic decides fault vs. expected event (e.g. force detect)
    using AlarmCallback = void(*)(StepperMotor &motor, void *context);
    void setAlarmCallback(AlarmCallback cb, void *ctx);
};
```

#### Behavior

- `go()` / `goTo()` check soft limits before execution, reject if out of bounds
- On ALM: invoke callback to upper layer; machine logic decides fault vs. expected event
- Position persistence: save to flash on fault/shutdown; at boot position is "probable" until re-home
- PEND timeout: if PEND not received within expected time, fault

---

### Cylinder

Two-position pneumatic actuator with confirmation. Positions A and B (no assumption
about which is "up"/"down" — semantics are defined in machine logic).

#### IO Signals

| Signal | Direction | Description |
|--------|-----------|-------------|
| Coil A | MCU → Valve | Solenoid A |
| Coil B | MCU → Valve | Solenoid B |
| Sensor A | Sensor → MCU | Position A confirmation |
| Sensor B | Sensor → MCU | Position B confirmation |

#### Configuration (per-instance)

```cpp
struct CylinderConfig {
    const gpio_dt_spec *coil_a;
    const gpio_dt_spec *coil_b;
    const gpio_dt_spec *sensor_a;
    const gpio_dt_spec *sensor_b;

    DriveMode drive_mode;     // PULSE or LEVEL
    uint32_t pulse_ms;        // for PULSE mode (e.g. 60ms)
    uint32_t timeout_ms;      // confirm timeout (production: 10s)
};
```

#### Drive Modes

- **PULSE** (self-latching, bistable valve, e.g. SY3220): short pulse, valve locks
  mechanically. Coils off in steady state.
- **LEVEL** (spring-return valve): coil stays energized. On coils off, spring returns
  to default position.

#### API

```cpp
class Cylinder {
public:
    int init();
    int goTo(Position pos);        // POS_A or POS_B, blocking, waits for confirm
    Position currentPos();         // POS_A, POS_B, or UNKNOWN
    void off();                    // coils off (safe state)
    void setTimeout(uint32_t ms);  // runtime override (commissioning = infinite)
};
```

#### Behavior

- `goTo(POS_A)` → de-energize coil_b, energize coil_a (pulse or level) → wait sensor_a with timeout
- Return: OK, TIMEOUT (fault), or INTERRUPTED
- Intermediate state (both sensors open) is normal during movement, fault if > timeout

#### Safe State

- PULSE mode: coils off = cylinder locked in last position
- LEVEL mode: coils off = spring returns to default position

---

### SmemaPort

IPC-9851 conveyor handshake interface. Separate component in the Component Layer —
standardized protocol, reusable between machines.

#### IO Signals

| Signal | Direction | Description |
|--------|-----------|-------------|
| Board Available (BA) | bidirectional | "I have a board ready" |
| Machine Ready (MR) | bidirectional | "I am ready to accept" |
| OK/NG routing | MCU → downstream | Test result routing signal |

#### API

```cpp
class SmemaPort {
public:
    // Upstream (receiving side)
    void setMachineReady(bool ready);
    bool boardAvailable();

    // Downstream (sending side)
    void offerBoard(TestResult result);  // BA + OK/NG routing
    bool machineReady();
    void clearOffer();
};
```

---

## SFC Engine (Machine Logic Layer)

### Execution Model

Scan-based cyclic execution (IEC 61131 SFC style), NOT thread-based blocking.

```cpp
void Engine::scan() {    // called every 5-10ms by a Zephyr timer/thread
    // 1. SAFETY — every scan, before anything else
    checkInterlocks();
    if (in_fault) return;

    // 2. Active action of current step
    if (current_step->on_active)
        current_step->on_active(ctx);

    // 3. In STEP mode — don't check transition without advance command
    if (mode == MODE_STEP && !advance_requested) return;

    // 4. Check transition
    if (current_step->transition(ctx)) {
        if (current_step->on_exit) current_step->on_exit(ctx);
        current_step = resolveNext(current_step);
        if (current_step->on_entry) current_step->on_entry(ctx);
    }
}
```

### Step Definition

```cpp
struct StepDef {
    const char *name;
    void (*on_entry)(Context &ctx);     // run once on step entry
    void (*on_active)(Context &ctx);    // run every scan while step is active (nullable)
    void (*on_exit)(Context &ctx);      // run once on step exit (nullable)
    bool (*transition)(Context &ctx);   // condition to advance
    int default_next;                   // next step index
    Branch *branches;                   // conditional jumps (nullable)
    int branch_count;
};

struct Branch {
    bool (*condition)(Context &ctx);
    int target_step;
};
```

### Context (shared state)

```cpp
struct Context {
    // Components
    StepperMotor &conveyor, &width, &table;
    Cylinder &stopper, &rfid, &locker;
    SmemaPort &smema_up, &smema_down;

    // Sensors (sequence triggers)
    Sensor &laser1, &laser2, &laser3;

    // Parameters (from active recipe)
    const Recipe &recipe;

    // Runtime state
    TestResult last_result;
    OperatingMode mode;
    uint32_t cycle_count;
    bool advance_requested;    // for STEP mode
    SoftTimer timer;           // scan-based timer (no k_msleep)
};
```

### Operating Modes

| Mode | Behavior |
|------|----------|
| AUTO | Continuous cycle: transitions advance automatically on condition |
| STEP | Transitions require explicit `advance_requested` flag (set by UART/CLI) |
| RETRY | On request: execute recovery procedure, jump to target checkpoint |
| REPETITIVE | After ejection: execute return procedure, restart cycle (N times or infinite) |

### Retry / Recovery

Retry is not "goto step N" — it is a configurable recovery procedure that safely
navigates the machine from current state to a target checkpoint.

```cpp
struct RetryPolicy {
    Checkpoint target;              // where to go back to
    const StepDef *recovery_seq;    // steps to get there safely
    int recovery_seq_len;
};
```

Each recovery procedure is machine-specific and respects safety (runs through the
same scan loop with interlocks active).

### Parallel Operations

Pseudo-parallel within a single step (sufficient for this machine class):

```cpp
{ "head_and_convey",
  .on_entry = [](ctx) {
      ctx.table.goTo(pos, rpm);     // non-blocking start
      ctx.conveyor.run(40, FWD);    // non-blocking start
  },
  .transition = [](ctx) {
      return !ctx.table.isMoving() && ctx.laser2.triggered();
  },
}
```

### Timers

Scan-based soft timers (no `k_msleep`). Safety scan continues during delays:

```cpp
{ "inter_step_delay",
  .on_entry = [](ctx) { ctx.timer.start(100); },
  .transition = [](ctx) { return ctx.timer.expired(); },
}
```

### Machine Selection

Sequences are compile-time, one `.cpp` file per machine. Selected via Kconfig:

```
choice MACHINE_TYPE
    prompt "Target machine"
config MACHINE_INFEED_TESTER
    bool "Infeed PCB tester"
config MACHINE_OTHER
    bool "Other machine"
endchoice
```

Future: DSL/code-generator or binary-table tooling can be added on top without
changing the engine.

---

## Safety

Safety is integrated into the scan loop — NOT a separate thread. Checked every scan
cycle before sequence logic runs.

### Checks (every scan)

1. Motor ALM signals — any alarm triggers callback
2. Cylinder timeouts — if goTo in progress and confirm overdue
3. Soft limit violations — belt-and-suspenders check
4. Watchdog feed — IWDG, if scan loop hangs → MCU reset

### Fault State Machine

```
NORMAL ──── alarm/timeout/violation ────→ FAULT
  ^                                         │
  │                                         v
  └──── operator reset + conditions met ── RECOVERY
```

- **NORMAL** — sequence runs, safety monitors
- **FAULT** — all motion stopped, outputs safe, sequence frozen. Operator reset required.
- **RECOVERY** — attempt to reach safe position (optional). If fails → stays in FAULT.

### ALM Dual Role

ALM can be either a fault OR an expected event (force detection when needles press
against a PCB). The engine invokes an alarm callback; machine logic decides based on
current step context:

- Step "lower_to_contact" + ALM → expected (force threshold reached)
- Any other step + ALM → fault

---

## Settings / Persistence

Hybrid approach using Zephyr Settings (ZMS/NVS backend):

### Storage Architecture

| Data | Write frequency | Format | Backend |
|------|----------------|--------|---------|
| Factory config | Once (assembly/maintenance) | Binary blob, versioned | NVS key "factory" |
| Recipes (N slots) | On product change (few/day) | Binary blob, versioned, CRC | NVS keys "recipe/0"..."recipe/7" |
| Active recipe index | On recipe switch | uint8 | NVS key "recipe/active" |
| Runtime state | On stop/fault | Individual keys | NVS keys "runtime/*" |
| Cycle count | Every 100 cycles (batched) | uint32 | NVS key "runtime/cycles" |

### Recipe Management

```cpp
class ConfigStore {
public:
    // Factory (Level 1)
    int loadFactory(FactoryConfig &out);
    int saveFactory(const FactoryConfig &cfg);

    // Recipes (Level 2)
    int recipeCount();
    int recipeList(char names[][32], int max);
    int loadRecipe(const char *name, Recipe &out);
    int saveRecipe(const char *name, const Recipe &recipe);
    int deleteRecipe(const char *name);
    int setActiveRecipe(const char *name);

    // Runtime
    int savePosition(int axis, int32_t steps);
    int loadPosition(int axis, int32_t &out);
    int saveCycleCount(uint32_t count);
};
```

### Version Migration

```cpp
struct Recipe {
    uint16_t version;       // incremented on struct change
    char name[32];
    // ... parameters ...
    uint16_t crc;           // integrity check
};

// On load: if version < CURRENT → migrate (fill new fields with defaults), save back
```

### Parameter Validation

```cpp
struct ParamDescriptor {
    const char *key;
    float min_value;
    float max_value;
    ConfigLevel required_level;  // FACTORY, RECIPE, OPERATOR
    const char *unit;            // "mm", "rpm", "ms"
};
```

All parameter writes are validated against bounds and access level before persisting.
