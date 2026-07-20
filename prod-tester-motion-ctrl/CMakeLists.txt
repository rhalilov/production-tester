cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(prod-tester-motion-ctrl LANGUAGES CXX C)

# Common sources (universal, machine-independent)
target_sources(app PRIVATE
    src/main.cpp
    src/app.cpp
    src/hal/gpio_pin.cpp
    src/components/stepper_motor.cpp
    src/components/cylinder.cpp
    src/components/smema_port.cpp
    src/components/sensor.cpp
    src/engine/engine.cpp
    src/engine/fault.cpp
    src/config/config_store.cpp
    src/cli/cli_machine.cpp
    src/cli/cli_manual.cpp
    src/cli/cli_config.cpp
    src/cli/cli_diag.cpp
    src/comm/tester_protocol.cpp
)

# Machine-specific sequence (selected by Kconfig)
if(CONFIG_MACHINE_INFEED_TESTER)
    target_sources(app PRIVATE src/machines/infeed_tester.cpp)
endif()

target_include_directories(app PRIVATE src)

target_compile_options(app PRIVATE
    -fno-exceptions
    -fno-rtti
    -Wall
    -Wextra
    -Wno-unused-parameter
)
