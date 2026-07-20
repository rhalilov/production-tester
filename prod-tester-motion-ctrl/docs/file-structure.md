# Project File Structure

## Directory Layout

```
prod-tester-motion-ctrl/
├── CMakeLists.txt                    # Zephyr project CMake
├── prj.conf                          # Kconfig (CONFIG_CPP, CONFIG_STD_CPP17, etc.)
├── Kconfig                           # Custom Kconfig (machine selection)
├── boards/
│   └── stm32f4_disco.overlay         # Device tree overlay (pins, peripherals)
│
├── docs/
│   ├── architecture.md               # Layered architecture design
│   ├── machine-process.md            # Process description & config levels
│   ├── cli-protocol.md               # CLI commands & UART protocol
│   └── file-structure.md             # This file
│
├── src/
│   ├── main.cpp                      # Entry point, init sequence
│   │
│   ├── hal/                          # Hardware Abstraction (thin wrappers)
│   │   ├── gpio_pin.h                # GPIO wrapper (dt_spec based)
│   │   ├── gpio_pin.cpp
│   │   ├── uart_port.h              # UART abstraction
│   │   └── uart_port.cpp
│   │
│   ├── components/                   # Component Layer (self-contained actuators)
│   │   ├── stepper_motor.h
│   │   ├── stepper_motor.cpp
│   │   ├── cylinder.h
│   │   ├── cylinder.cpp
│   │   ├── smema_port.h
│   │   ├── smema_port.cpp
│   │   └── sensor.h                 # Debounced digital input with configurable hold time
│   │
│   ├── engine/                       # SFC Engine (universal, machine-independent)
│   │   ├── engine.h                  # Scan loop, step execution, safety checks
│   │   ├── engine.cpp
│   │   ├── step.h                   # StepDef, Branch, Context structs
│   │   ├── timer.h                  # Soft timer (scan-based, no k_msleep)
│   │   └── fault.h                  # Fault state machine (NORMAL/FAULT/RECOVERY)
│   │
│   ├── config/                       # Settings & Persistence
│   │   ├── config_store.h           # Factory/Recipe/Runtime storage API
│   │   ├── config_store.cpp
│   │   ├── recipe.h                 # Recipe struct + version migration
│   │   ├── factory_config.h         # Factory calibration struct
│   │   └── param_descriptor.h       # Parameter metadata (bounds, unit, level)
│   │
│   ├── cli/                          # Command Line Interface
│   │   ├── cli.cpp                  # Zephyr shell registration, command dispatch
│   │   ├── cli_machine.cpp          # start/stop/abort/reset/mode commands
│   │   ├── cli_manual.cpp           # motor/cylinder direct commands
│   │   ├── cli_config.cpp           # recipe/set/cal/param commands
│   │   ├── cli_diag.cpp            # status/sensors/log/version commands
│   │   └── auth.h                   # Permission levels, session management
│   │
│   ├── comm/                         # Tester Communication (USART3)
│   │   ├── tester_protocol.h        # Command/event definitions
│   │   └── tester_protocol.cpp      # Parse commands, emit events
│   │
│   └── machines/                     # Machine-specific sequences (one per machine)
│       ├── infeed_tester.h          # Hardware instances (motors, cylinders, pins)
│       ├── infeed_tester.cpp        # Sequence definition (StepDef array)
│       └── README.md                # How to add a new machine
```

## Scope: Universal vs. Per-Machine

| Directory | Scope | Changes for new machine? |
|-----------|-------|--------------------------|
| `hal/` | Universal | No (unless new MCU) |
| `components/` | Universal | No |
| `engine/` | Universal | No |
| `config/` | Universal | Recipe struct may grow (version migration handles it) |
| `cli/` | Universal | No |
| `comm/` | Universal | No |
| `machines/` | **Per-machine** | **Yes** — new file for each machine |
| `boards/` | **Per-hardware** | **Yes** — new overlay for different board/pin layout |

## Build: Machine Selection

### Kconfig

```kconfig
choice MACHINE_TYPE
    prompt "Target machine"
    default MACHINE_INFEED_TESTER

config MACHINE_INFEED_TESTER
    bool "Infeed PCB tester"

config MACHINE_OTHER
    bool "Other machine (example)"

endchoice
```

### CMakeLists.txt (excerpt)

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(prod-tester-motion-ctrl LANGUAGES CXX)

# Common sources (universal)
target_sources(app PRIVATE
    src/main.cpp
    src/hal/gpio_pin.cpp
    src/hal/uart_port.cpp
    src/components/stepper_motor.cpp
    src/components/cylinder.cpp
    src/components/smema_port.cpp
    src/engine/engine.cpp
    src/config/config_store.cpp
    src/cli/cli.cpp
    src/cli/cli_machine.cpp
    src/cli/cli_manual.cpp
    src/cli/cli_config.cpp
    src/cli/cli_diag.cpp
    src/comm/tester_protocol.cpp
)

# Machine-specific sequence
if(CONFIG_MACHINE_INFEED_TESTER)
    target_sources(app PRIVATE src/machines/infeed_tester.cpp)
elseif(CONFIG_MACHINE_OTHER)
    target_sources(app PRIVATE src/machines/other_machine.cpp)
endif()

# C++ flags
target_compile_options(app PRIVATE -fno-exceptions -fno-rtti)
```

### prj.conf

```ini
# C++ support
CONFIG_CPP=y
CONFIG_STD_CPP17=y

# Shell (CLI)
CONFIG_SHELL=y
CONFIG_SHELL_BACKEND_SERIAL=y

# USB CDC
CONFIG_USB_DEVICE_STACK=y
CONFIG_USB_CDC_ACM=y

# Settings persistence
CONFIG_SETTINGS=y
CONFIG_SETTINGS_NVS=y
CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
CONFIG_NVS=y

# Watchdog
CONFIG_WATCHDOG=y

# Logging
CONFIG_LOG=y
CONFIG_LOG_BACKEND_UART=y

# Machine selection
CONFIG_MACHINE_INFEED_TESTER=y
```

## Adding a New Machine

1. Create `src/machines/my_machine.h` — define hardware instances (which pins, how
   many motors/cylinders, sensor assignments)
2. Create `src/machines/my_machine.cpp` — define the `StepDef` sequence array, retry
   policies, and checkpoint definitions
3. Add a `config MACHINE_MY_MACHINE` entry to `Kconfig`
4. Add the corresponding `target_sources` line to `CMakeLists.txt`
5. If different board: create a new `.overlay` in `boards/`
6. Build with `west build -b stm32f4_disco -- -DCONFIG_MACHINE_MY_MACHINE=y`
