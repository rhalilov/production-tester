# Architecture — prod-tester-motion-ctrl

## Решения

- **Език:** C++ в "C with classes" стил (`-fno-exceptions`, `-fno-rtti`, без STL)
- **Zephyr:** `CONFIG_CPP=y`, `CONFIG_STD_CPP17=y`, `extern "C"` обвивки за Zephyr callbacks
- **MCU:** STM32F407 (STM32F4 Discovery board)
- **RTOS:** Zephyr v4.x

---

## Layered Architecture

```
┌─────────────────────────────────────────────────────────┐
│  User Interface                                          │
│  CLI (Zephyr Shell) + Event Logger                       │
│  USB-CDC Console │ USART3 Console                        │
├─────────────────────────────────────────────────────────┤
│  Machine Logic Layer                                     │
│  - Production cycle (32 стъпки)                          │
│  - Sequence triggers: Laser1, Laser2, Laser3             │
│  - SMEMA handshake                                       │
│  - Семантика: STOPPER_UP=POS_A, test_position=-42000     │
│  - ALM контекст: fault vs. force detect                  │
├─────────────────────────────────────────────────────────┤
│  Component Layer                                         │
│  ┌─────────────────────┐  ┌──────────────────────┐      │
│  │  StepperMotor        │  │  Cylinder             │     │
│  │  - position tracking │  │  - goTo(POS_A/POS_B)  │     │
│  │  - soft limits       │  │  - confirm sensors    │     │
│  │  - ALM monitoring    │  │  - timeout            │     │
│  │  - PEND (optional)   │  │  - PULSE / LEVEL mode │     │
│  │  - homing (optional) │  │                       │     │
│  └─────────────────────┘  └──────────────────────┘      │
├─────────────────────────────────────────────────────────┤
│  HAL / IO                                                │
│  GPIO Pins, Zephyr Stepper Driver, ZMS Settings          │
└─────────────────────────────────────────────────────────┘
```

---

## Component Layer

### StepperMotor

Самозащитен компонент. Не взима process-level решения, но enforce-ва собствени
граници и реагира на аларми.

#### IO сигнали

| Сигнал | Посока | Описание |
|--------|--------|----------|
| STEP | MCU → Driver | Стъпков pulse |
| DIR | MCU → Driver | Посока |
| ENA | MCU → Driver | Enable (active low, HBS57) |
| ALM | Driver → MCU | Alarm (fault или force threshold) |
| PEND | Driver → MCU | Positioning complete (optional) |

#### Конфигурация per-instance

```cpp
struct StepperMotorConfig {
    // Outputs
    const gpio_dt_spec *step_pin;
    const gpio_dt_spec *dir_pin;
    const gpio_dt_spec *ena_pin;

    // Inputs — multiple за dual-driver оси (масата)
    const gpio_dt_spec *alm_pins;
    uint8_t alm_count;              // 1 за конвейер/ширина, 2 за масата
    AlmPolicy alm_policy;           // ANY_TRIGGERS_FAULT

    const gpio_dt_spec *pend_pins;
    uint8_t pend_count;             // 0 за конвейер/ширина, 2 за масата
    PendPolicy pend_policy;         // ALL_REQUIRED

    // Homing (optional)
    bool has_home;
    const gpio_dt_spec *home_sensor;
    MotionDir home_dir;
    bool auto_home;                 // false за width (опасно при boot)
    uint32_t home_rpm;

    // Soft limits (optional, само за позиционни оси)
    bool has_limits;
    int32_t soft_limit_min;         // стъпки от home
    int32_t soft_limit_max;
    int32_t safe_position;          // go-to при alarm recovery

    // Position persistence
    bool persist_position;          // записвай позиция при shutdown/fault
};
```

#### Инстанции в тази машина

| Ос | Мотори | ALM | PEND | Home | Soft limits | Бележки |
|----|--------|-----|------|------|-------------|---------|
| Axis1 (конвейер) | 1 | 1 | няма | няма | няма | Непрекъснат, спира по команда от machine logic |
| Axis2 (ширина) | 1 | 1 | няма | има | има | auto_home = false (платка може да е на конвейра) |
| Axis3 (маса/игли) | 2 (споделен STEP/DIR/ENA) | 2 | 2 | има (Photo10) | има | auto_home = true при boot |

#### API

```cpp
class StepperMotor {
public:
    int init();
    int home();                                    // еднократно, ако has_home
    int go(int32_t steps, uint32_t rpm);           // блокиращ, чака PEND ако има
    int goTo(int32_t position, uint32_t rpm);      // абсолютна позиция (от home)
    int run(uint32_t rpm, MotionDir dir);          // непрекъснат
    void stop();
    void emergencyStop();                          // без decel

    // State
    bool isMoving();
    int32_t position();                            // steps from home
    bool isHomed();
    bool inAlarm();

    // Callback — machine logic решава fault vs. expected event
    using AlarmCallback = void(*)(StepperMotor &motor, void *context);
    void setAlarmCallback(AlarmCallback cb, void *ctx);
};
```

#### Поведение

- `go()` / `goTo()` проверяват soft limits преди изпълнение, отказват ако извън граници
- При ALM — извиква callback нагоре; machine logic решава дали е fault или очакван event (force detect)
- Position persistence — записва позиция на flash при fault/shutdown; при boot позицията е "probable" до re-home
- PEND timeout — ако PEND не дойде в очакваното време → fault

---

### Cylinder

Двупозиционен пневматичен актуатор с потвърждение. Позиции A и B (без допускания коя е "горе"/"долу" — семантиката е в machine logic).

#### IO сигнали

| Сигнал | Посока | Описание |
|--------|--------|----------|
| Coil A | MCU → Valve | Соленоид A |
| Coil B | MCU → Valve | Соленоид B |
| Sensor A | Sensor → MCU | Потвърждение позиция A |
| Sensor B | Sensor → MCU | Потвърждение позиция B |

#### Конфигурация per-instance

```cpp
struct CylinderConfig {
    const gpio_dt_spec *coil_a;
    const gpio_dt_spec *coil_b;
    const gpio_dt_spec *sensor_a;
    const gpio_dt_spec *sensor_b;

    DriveMode drive_mode;     // PULSE или LEVEL
    uint32_t pulse_ms;        // за PULSE mode (напр. 60ms)
    uint32_t timeout_ms;      // confirm timeout (production: 10s)
};
```

#### Режими на управление

- **PULSE** (self-latching, bistable valve, напр. SY3220): кратък импулс, клапанът се заключва механично. Coils off в steady state.
- **LEVEL** (spring-return valve): бобината остава захранена. При coils off — пружина връща в default.

#### API

```cpp
class Cylinder {
public:
    int init();
    int goTo(Position pos);        // POS_A или POS_B, блокиращ, чака confirm
    Position currentPos();         // POS_A, POS_B, или UNKNOWN
    void off();                    // coils off (safe state)
    void setTimeout(uint32_t ms);  // runtime override (commissioning = infinite)
};
```

#### Поведение

- `goTo(POS_A)` → de-energize coil_b, energize coil_a (pulse или level) → чакай sensor_a с timeout
- Return: OK, TIMEOUT (fault), или INTERRUPTED
- Междинно състояние (и двата сензора отворени) е нормално по време на движение, fault ако > timeout

#### Safe state

- PULSE mode: coils off = цилиндърът заключен в последна позиция
- LEVEL mode: coils off = пружината го връща в default позиция

---

## Machine Logic Layer

Оркестрира компонентите без да знае вътрешните им детайли. Знае за:

- Sequence trigger сензори (Laser1, Laser2, Laser3) — НЕ принадлежат на actuator
- SMEMA handshake сигнали
- Production cycle последователност (стъпки 1-32)
- Семантика на позициите (STOPPER_UP = POS_A, и т.н.)
- Контекст за ALM: "ако получа alarm при стъпка #10 = force detect, не fault"

```cpp
// Machine logic дефинира семантиката:
constexpr auto STOPPER_UP = Cylinder::POS_A;
constexpr auto STOPPER_DOWN = Cylinder::POS_B;

// Процесът работи с високо-ниво операции:
conveyor.run(40, FORWARD);
waitUntil(laser2);
conveyor.stop();
stopper.goTo(STOPPER_UP);
table.goTo(testPosition);
```

---

## Теми за бъдещо обсъждане

- Machine Logic layer архитектура (state machine vs. sequential thread)
- Settings / persistence: per-component настройки (timeout, rpm, limits)
- CLI интерфейс: как shell командите се адаптират към новата структура
- Safety supervisor: как се вписва в новите layer-и (глобален fault координатор?)
- SMEMA: отделен модул или част от machine logic?
