# Machine Process Description

## Overview

The prod-tester-motion-ctrl is a **universal motion controller** for production test
fixtures. While this document describes one specific machine (PCB infeed tester), the
controller is designed to drive any similar fixture by reprogramming the process
sequence. Once programmed for a given machine, operators can adjust parameters (speeds,
positions, timings) without risk of breaking the underlying logic.

### Design Principles

1. **Programmable** — the production sequence is machine-specific and can be redefined
   for different fixtures without modifying the controller firmware core.
2. **Parameterizable** — once a sequence is defined, operators tune speeds, positions,
   and timings through a safe configuration interface. Parameters have min/max bounds
   enforced by the system.
3. **Multi-mode** — supports automatic, step-by-step, retry, and repetitive operation
   within the same sequence definition.
4. **Multi-level configuration** — three distinct configuration levels with different
   access requirements (see Configuration Levels below).

---

## Configuration Levels

The system has three levels of configuration, each with different access rights and
performed at different stages of the machine lifecycle:

### Level 1: Factory Calibration

**Performed by:** Machine builder / maintenance technician  
**When:** After machine assembly, after mechanical maintenance, after sensor replacement  
**Access:** Protected (requires service password or hardware jumper)

Defines the physical geometry of the machine — where sensors are physically mounted,
mechanical travel limits, and reference offsets. These values change only when the
hardware changes.

| Parameter | Unit | Description |
|-----------|------|-------------|
| home_sensor_offset_axis2 | mm | Distance from Axis2 home sensor to mechanical zero |
| home_sensor_offset_axis3 | mm | Distance from Axis3 home sensor to mechanical zero |
| laser1_position | mm | Entry laser mounting position along conveyor |
| laser2_position | mm | Middle laser mounting position along conveyor |
| laser3_position | mm | Exit laser mounting position along conveyor |
| axis2_travel_min | mm | Minimum width (mechanical limit) |
| axis2_travel_max | mm | Maximum width (mechanical limit) |
| axis3_travel_min | mm | Maximum head descent (mechanical limit) |
| axis3_travel_max | mm | Maximum head ascent (mechanical limit) |
| steps_per_mm_axis1 | steps/mm | Conveyor linear calibration |
| steps_per_mm_axis2 | steps/mm | Width axis linear calibration |
| steps_per_mm_axis3 | steps/mm | Head axis linear calibration |

These values are stored in flash and survive firmware updates.

### Level 2: Product Setup (Recipe)

**Performed by:** Process engineer  
**When:** When introducing a new PCB type, or modifying test parameters  
**Access:** Engineer-level password

Defines all parameters for a specific PCB product. A machine can store multiple
recipes (one per PCB type) and switch between them.

| Parameter | Unit | Description |
|-----------|------|-------------|
| board_width | mm | Conveyor width for this PCB |
| use_rfid | bool | Whether bottom RFID mechanism is used |
| pos_prepin | mm | Head pre-pin position (centering pins clear of board) |
| pos_pin_entry | mm | Head position where centering pins engage |
| pos_full_contact | mm | Head position for full needle pressure |
| fast_approach_rpm | rpm | Head speed: home/park → pre-pin |
| fast_approach_accel | rpm/s | Head acceleration for fast phase |
| pin_engage_rpm | rpm | Head speed: pre-pin → pin-entry |
| pin_engage_accel | rpm/s | Head acceleration for pin engagement |
| contact_rpm | rpm | Head speed: pin-entry → full-contact |
| contact_accel | rpm/s | Head acceleration for contact phase |
| convey_rpm | rpm | Conveyor infeed speed |
| convey_accel | rpm/s | Conveyor acceleration |
| creep_rpm | rpm | Conveyor slow approach speed |
| creep_distance | mm | Distance from middle laser to stopper contact |
| outfeed_rpm | rpm | Conveyor outfeed speed |
| eject_extra | mm | Extra travel after exit laser (ensure board fully out) |
| retry_reverse | mm | Reverse distance for retry mode |
| laser_debounce_ms | ms | Hold time for laser triggers (ignore holes in PCB) |
| test_timeout_s | s | Maximum time to wait for test result from tester |
| repetitive_count | int | Number of cycles in repetitive mode (0 = infinite) |

**Laser debounce:** PCBs can have holes or cutouts that momentarily break the laser
beam. The `laser_debounce_ms` parameter defines how long the laser must remain
continuously triggered before the system accepts it as a valid "board present" event.
This prevents false triggers from holes passing through the beam.

### Level 3: Operator Adjustment

**Performed by:** Production operator  
**When:** During production, for fine-tuning  
**Access:** No password required (limited parameter set)

Operators can only adjust a subset of parameters within bounds defined by the process
engineer in Level 2. Typical operator-accessible parameters:

- Conveyor speed (within ±20% of recipe value)
- Test timeout extension
- Repetitive cycle count

All other parameters are read-only for operators. The system displays current values
but rejects modification attempts outside the allowed set or range.

---

## Motor Speed and Acceleration Profiles

All motors (not just the head) support configurable speed AND acceleration per motion
segment. In other machines, motors may serve entirely different purposes (rotary
tables, pick-and-place axes, press mechanisms), but the principle is the same:

Each motion command specifies:
- **Target speed (rpm)** — the steady-state velocity
- **Acceleration (rpm/s)** — how quickly to reach target speed (ramp-up/ramp-down)

This allows smooth motion profiles:
- Conveyor: gentle start to avoid board slippage, gentle stop to avoid overshoot
- Head: fast approach with sharp deceleration near the board
- Width: slow, controlled move to avoid damage

The acceleration parameter is part of the recipe (Level 2) and bounded by factory
calibration (Level 1) limits that protect the mechanical system.

---

## Operating Modes

| Mode | Description |
|------|-------------|
| AUTO | Normal production cycle: board in → test → board out. Continuous loop. |
| STEP | Same sequence as AUTO but each step is advanced by an external command (UART/CLI). Used for commissioning and debugging. |
| RETRY | On operator request or test failure: reverse the board by a configurable distance, re-engage, and repeat the test portion. |
| REPETITIVE | Board is tested and returned to the start instead of being ejected. Runs N cycles or indefinitely. Used for reliability/burn-in testing. |

---

## Setup Process (Commissioning)

Before automatic operation, the machine must be configured for the specific PCB and
test head. The following parameters are set during commissioning:

### Board Parameters

| Parameter | Unit | Description |
|-----------|------|-------------|
| board_width | mm | Conveyor width adjustment (Axis2 position) |
| use_rfid | bool | Whether the bottom RFID plate mechanism is active |

### Head Positions

The test head (Axis3) operates at multiple named positions, each configurable:

```
Home (top limit, only at boot)
  │
  │  fast_rpm
  ▼
Pre-pin position ─── "safe travel" point; centering pins NOT in board.
  │                   The cycle parks here between tests. Board can
  │  slow_rpm         enter/exit freely at this height.
  ▼
Pin-entry position ── centering pins engaged, needles touching PCB.
  │                   No downward pressure yet.
  │  contact_rpm
  ▼
Full-contact position ── additional mm below pin-entry for needle
                         pressure. Ensures reliable electrical contact.
```

| Parameter | Unit | Description |
|-----------|------|-------------|
| pos_prepin | steps | Pre-pin position (from home). Centering pins clear of board. |
| pos_pin_entry | steps | Pin-entry position. Pins in, needles touching, no pressure. |
| pos_full_contact | steps | Full contact. Additional depth for needle pressure. |

### Speed Profile (Head Descent)

The head descent uses a multi-phase speed profile:

| Phase | From → To | Speed | Rationale |
|-------|-----------|-------|-----------|
| Fast approach | Home/park → pre-pin | fast_rpm | No mechanical contact, maximize throughput |
| Pin engagement | pre-pin → pin-entry | slow_rpm | Centering pins entering board, must be gentle |
| Needle contact | pin-entry → full-contact | contact_rpm | Needles contacting pads, controlled force |

### Conveyor Parameters

| Parameter | Unit | Description |
|-----------|------|-------------|
| convey_rpm | rpm | Infeed speed (entry laser → middle laser) |
| creep_rpm | rpm | Slow speed after middle laser, approaching stopper |
| creep_steps | steps | Distance after middle laser trigger to stopper |
| outfeed_rpm | rpm | Speed for board ejection |
| eject_extra_steps | steps | Extra travel after exit laser to ensure board fully out |
| retry_reverse_mm | mm | Reverse distance for retry mode |

---

## Automatic Operation Process

The following describes the full automatic cycle for one board. Steps are grouped
into logical phases.

### Phase 1: Board Intake

| Step | Action | Condition to advance |
|------|--------|---------------------|
| 1.1 | Signal upstream SMEMA: Machine Ready. Wait for Board Available. | SMEMA upstream Board Available = HIGH |
| 1.2 | Assert SMEMA upstream acceptance. Start conveyor forward at `convey_rpm`. | Entry laser (Laser1) triggered (board entering) |
| 1.3 | Board passing through entry laser. Continue conveyor. | Entry laser cleared (entire board has passed through) |
| 1.4 | De-assert SMEMA upstream acceptance. Raise stopper (cylinder to POS_A). Wait for stopper confirm. | Stopper confirm sensor = reached |
| 1.5 | Continue conveyor at `convey_rpm`. Wait for middle laser. | Middle laser (Laser2) triggered |
| 1.6 | Count `creep_steps` at `creep_rpm`. Board approaches stopper at reduced speed. | Step count complete (board resting against stopper) |
| 1.7 | Stop conveyor. | Immediate |

### Phase 2: Head Descent and Engagement

| Step | Action | Condition to advance |
|------|--------|---------------------|
| 2.1 | Move head (Axis3) to `pos_prepin` at `fast_rpm`. | Move complete (PEND or step count) |
| 2.2 | Move head to `pos_pin_entry` at `slow_rpm`. Centering pins engage board. | Move complete |
| 2.3 | Engage board lock (cylinder to lock position). Wait for confirm. | Lock confirm sensor |
| 2.4 | If `use_rfid`: engage RFID plate (cylinder up from below). Wait for confirm. | RFID confirm sensor (or skip if disabled) |
| 2.5 | Move head to `pos_full_contact` at `contact_rpm`. Full needle pressure. | Move complete |

### Phase 3: Testing

| Step | Action | Condition to advance |
|------|--------|---------------------|
| 3.1 | Send UART command: "start test". | UART acknowledgment received |
| 3.2 | Wait for test result via UART. | Result received: PASS, FAIL, or RETRY_REQUEST |
| 3.3 | Store test result (OK/NG) for SMEMA output routing. | Immediate |

### Phase 4: Head Retract and Disengagement

| Step | Action | Condition to advance |
|------|--------|---------------------|
| 4.1 | Move head up to `pos_pin_entry` at `contact_rpm`. Relieve needle pressure; centering pins still in board. | Move complete |
| 4.2 | If `use_rfid`: retract RFID plate (cylinder down). Wait for confirm. | RFID confirm sensor |
| 4.3 | Move head up to `pos_prepin` at `slow_rpm`. Extract centering pins from board. | Move complete |
| 4.4 | Release board lock (cylinder to unlock position). Wait for confirm. | Unlock confirm sensor |

### Decision Point: Retry or Continue

At this point, if the operator requests a retry (via UART or panel):

| Step | Action | Condition to advance |
|------|--------|---------------------|
| R.1 | Lower stopper (cylinder to POS_B). | Stopper confirm |
| R.2 | Reverse conveyor by `retry_reverse_mm`. | Move complete |
| R.3 | → Jump back to Step 1.4 (raise stopper and re-position) | — |

In REPETITIVE mode, after ejection the board returns to the start:

| Step | Action | Condition to advance |
|------|--------|---------------------|
| REP.1 | Reverse conveyor until entry laser triggers. | Entry laser triggered |
| REP.2 | → Jump back to Step 1.4 | — |

### Phase 5: Board Ejection

| Step | Action | Condition to advance |
|------|--------|---------------------|
| 5.1 | Lower stopper (cylinder to POS_B). Wait for confirm. | Stopper confirm |
| 5.2 | Wait for UART command to eject (optional, mode-dependent). | Command received or AUTO mode |
| 5.3 | Start conveyor forward at `outfeed_rpm`. | Exit laser (Laser3) triggered |
| 5.4 | Signal downstream SMEMA: Board Available (with OK/NG routing). | Downstream Machine Ready = HIGH |
| 5.5 | Continue conveyor until board clears exit laser + `eject_extra_steps`. | Exit laser cleared + extra steps complete |
| 5.6 | Stop conveyor. De-assert downstream Board Available. | Immediate |
| 5.7 | → Return to Step 1.1 (ready for next board) | — |

---

## Step-by-Step Mode (STEP)

Identical sequence to AUTO, but:
- Each step waits for an explicit "advance" command via UART/CLI before executing.
- The operator can inspect machine state between steps.
- Timeouts are disabled (no alarm on slow cylinder/motor response).
- Useful for commissioning, debugging, and verifying sensor wiring.

---

## Safety and Interlocks

The following conditions are checked continuously (every scan cycle), regardless of
the current step:

| Condition | Action |
|-----------|--------|
| Any motor ALM signal | Stop all motion, retract head to safe position if possible, latch fault |
| Cylinder timeout (confirm not received) | Stop sequence, latch fault |
| Head position exceeds soft limits | Emergency stop on Axis3 |
| E-stop (external, if wired) | All outputs off, all motion stopped |

Fault recovery requires explicit operator action (reset command via CLI/UART).

---

## UART Interface

Text-based command/response protocol (CLI-style). Used for:

1. **MCU → Tester:** notifications (test start, board positioned, cycle state)
2. **Tester → MCU:** commands (start test result, retry request, advance step)
3. **Operator → MCU:** configuration, manual commands, mode selection

Protocol details to be defined. Shared with the interactive CLI (same command parser,
different transport: USB-CDC for operator console, USART for tester communication).

---

## Parameterization Safety

Operators can modify parameters within bounds defined by the machine programmer:

```cpp
struct Parameter {
    int32_t value;
    int32_t min_bound;    // enforced minimum
    int32_t max_bound;    // enforced maximum
    const char *name;
    const char *unit;
};
```

- The sequence logic is **read-only** for operators — they cannot add, remove, or
  reorder steps.
- Parameters have hard min/max bounds set during machine programming.
- Invalid parameter values are rejected with an error message.
- Parameter changes take effect on the next cycle (not mid-cycle).
- All parameter changes are logged for traceability.
