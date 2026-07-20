# CLI and Communication Protocol

## Physical Interfaces

| Interface | Hardware | Role |
|-----------|----------|------|
| USB-CDC | USB connector | Operator/engineer CLI (configuration, debug, manual commands) |
| USART3 (PD8/PD9) | Pin header | Tester communication + firmware programming |

USART3 is dual-purpose:
- **Runtime:** communication with the tester (test start/result, events, step control)
- **Programming:** firmware flashing via STM32 bootloader (RTS/DTR drive BOOT0/NRST,
  Espressif-style auto-reset circuit)

Both interfaces share the same command parser. Permissions may differ by interface.

---

## Message Format

Line-based ASCII protocol:

```
Command:   <CMD> [ARG1] [ARG2]...\r\n
Response:  OK [data]\r\n
      or:  ERROR <code> <message>\r\n
Event:     EVT <type> [data]\r\n
```

Why ASCII (not binary):
- Debug with any terminal (PuTTY, minicom, screen)
- Tester can be a Python script — easy to parse
- No speed requirement (commands are seconds apart, not microseconds)

---

## Tester Protocol (USART3)

### Tester → MCU (commands)

```
TEST START              Start the test (MCU responds OK when ready)
TEST RESULT PASS        Test passed — board is OK
TEST RESULT FAIL        Test failed — board is NG
TEST RESULT RETRY       Request retry (re-position and re-test)
STEP ADVANCE            Advance one step (STEP mode only)
CYCLE ABORT             Abort current cycle
```

### MCU → Tester (unsolicited events)

```
EVT BOARD_READY         Board positioned, needles engaged, ready for test
EVT CYCLE_DONE PASS     Cycle complete, board passed
EVT CYCLE_DONE FAIL     Cycle complete, board failed
EVT FAULT <reason>      Machine fault occurred
EVT STEP <step_name>    Current step changed (in STEP mode)
EVT MODE <mode>         Operating mode changed
```

---

## CLI Commands (USB-CDC + USART3)

### Permission Levels

| Level | Access | How to obtain |
|-------|--------|---------------|
| operator | Basic control, status, recipe load | Default (no auth required) |
| engineer | Manual actuators, recipe save, mode change | `auth <engineer_password>` |
| factory | Calibration, reboot, bootloader | `auth <factory_password>` |

Session-based: permission resets on disconnect or idle timeout (30 min).

### Machine Control

```
start                         [operator]  Start/resume auto cycle (re-home + run)
stop                          [operator]  Pause sequence (enter manual mode)
abort                         [operator]  Emergency stop (latch fault, all outputs off)
reset                         [operator]  Clear fault, re-home
mode auto                     [operator]  Switch to automatic mode
mode step                     [operator]  Switch to step-by-step mode
mode repetitive [N]           [engineer]  Repetitive test (N cycles, 0=infinite)
```

### Step Control (STEP/MANUAL mode only)

```
step next                     [operator]  Advance one step
step goto <name>              [engineer]  Jump to named step (with recovery)
step current                  [operator]  Show current step name and index
step list                     [operator]  List all steps, mark current
```

### Manual Actuators (MANUAL mode only)

```
motor <1-3> run <rpm> <fwd|rev>  [engineer]  Continuous run
motor <1-3> stop                 [engineer]  Stop motor
motor <1-3> go <steps> <rpm>     [engineer]  Relative move (steps)
motor <1-3> goto <mm>            [engineer]  Absolute move (mm from home)
motor <1-3> home                 [engineer]  Run homing procedure
cylinder <1-3> a|b               [engineer]  Move to position A or B
cylinder <1-3> off               [engineer]  Coils off
```

Refused with error if sequence is running in AUTO mode:
```
> motor 3 goto -10.0
ERROR 403 sequence running - use 'stop' first
```

### Diagnostics

```
status                        [operator]  Machine state, step, mode, faults
sensors                       [operator]  All sensor states (snapshot)
sensors watch                 [engineer]  Live sensor update (Ctrl+C to stop)
log [on|off]                  [operator]  Enable/disable cycle event log
version                       [operator]  Firmware version, build date, machine type
uptime                        [operator]  Runtime, cycle count, last fault
```

#### `sensors` output example

```
SENSOR         PIN    STATE  ROLE              DEBOUNCE
laser1         PB0    HIGH   panel_presented   50ms
laser2         PB1    LOW    panel_near        50ms
laser3         PB2    LOW    panel_in_position 50ms
inductive4     PC4    LOW    cyl1_a            -
inductive5     PC5    LOW    cyl1_b            -
inductive6     PA4    HIGH   cyl2_a            -
inductive7     PA5    LOW    cyl2_b            -
inductive8     PA6    LOW    cyl3_a            -
inductive9     PA7    LOW    cyl3_b            -
photo10        PC8    HIGH   table_home        -
photo11        PC9    LOW    (unassigned)      -

ALM            PIN    STATE
alm1           PE0    OK
alm2           PE1    OK
alm3a          PE2    OK
alm3b          PE3    OK

PEND           PIN    STATE
pend3a         PD0    IDLE
pend3b         PD1    IDLE

SMEMA          DIR    PIN    STATE
up_BA          in     PC1    LOW
up_MR          out    PC2    HIGH
down_MR        in     PA1    LOW
down_BA        out    PC3    LOW
```

### Recipe Management

```
recipe list                   [operator]  Show saved recipes
recipe load <name>            [engineer]  Load recipe (all Level 2 params)
recipe save <name>            [engineer]  Save current params as recipe
recipe delete <name>          [engineer]  Delete a recipe
recipe show                   [operator]  Show active recipe name and all params
```

### Parameters

```
set <param> <value>           [operator]  Set operator-level parameter (bounded)
cal <param> <value>           [factory]   Set factory calibration parameter
param list                    [operator]  List all params with values, units, bounds
param list factory            [factory]   List factory-level params
```

### System

```
auth <password>               [-]         Elevate permission level
save                          [engineer]  Persist current config to flash
reboot                        [factory]   Software reset
bootloader                    [factory]   Enter DFU/bootloader mode (USART3)
```

---

## USART3 Hardware: Boot/Reset Circuit

For firmware programming without manual button presses, USART3 RTS and DTR lines
control the STM32 BOOT0 and NRST pins via a transistor circuit (Espressif-style):

```
USB-UART RTS ──→ transistor circuit ──→ STM32 BOOT0
USB-UART DTR ──→ transistor circuit ──→ STM32 NRST
```

The `bootloader` CLI command or the PC programming tool asserts the correct
RTS/DTR sequence to enter the STM32 system bootloader automatically.
