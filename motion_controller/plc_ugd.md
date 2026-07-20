# INFEED Controller — User Guide

STM32F407 "PLC" for the Shelly **Main** infeed test fixture. Runs the 21‑step
per‑panel cycle (homing → infeed → clamp → needle‑test → release → outfeed →
SMEMA hand‑off), looping continuously. Sensor inputs are bound to **logical
roles** that can be re‑assigned from the console and saved to flash, so the same
firmware adapts to different fixtures/wiring without reflashing.

- Pin map: `hw_docs/PIN_map_v5.csv` · Behaviour spec: `tester_emul_logic.md`
- Bench input emulator (L476): `emul_ugd.md`

---

## 1. Connecting to the console

- **Primary:** USB‑CDC‑ACM on the F407 **OTG‑FS** micro‑USB (CN5, pins PA11/PA12).
  Enumerates as a serial port (Windows `COMx`, Linux `/dev/ttyACMx`), **115200 8N1**.
- **Fallback:** USART3 **TX=PD8 / RX=PD9** via an external 3.3 V USB‑TTL adapter.
- Shell prompt is `uart:~$`. All commands below are sub‑commands of **`infeed`**.
  (The stock `stepper`, `kernel`, `device`, `log` shells are also available.)

A **configured** unit (one with a saved config) **auto‑starts at boot** — it homes,
then idles waiting for the upstream SMEMA board. A **never‑configured** unit
**safe‑holds** at boot (no motion) until you run `infeed start` (§4.6). You
drive/observe everything from the console.

---

## 2. Default wiring

> Levels: sensors are **PNP, active‑HIGH** (source‑high when triggered). Drive
> feedback ALM/PEND are **open‑collector, active‑LOW** (pulled up). Motor **ENA is
> active‑LOW** (logical *enabled* = pin LOW). All pins 5 V‑tolerant unless noted.

### 2.1 Motor drives — 3 step/dir banks → 4 closed‑loop step‑servos

Banks 1 and 2 drive one motor each; **bank 3 drives motors 3 & 4 synchronously**
(shared pulse train). Each drive reports its own PEND+ (in‑position) and ALM+ (alarm).

| Bank | Motor / role | PLS‑ (step) | DIR‑ | ENA‑ | PEND+ | ALM+ |
|---|---|---|---|---|---|---|
| 1 | Motor 1 — **conveyor** | PE5 | PE2 | PB8 | PE6 | PE4 |
| 2 | Motor 2 — **width** (not in cycle yet) | PB7 | PD6 | PD3 | PB9 | PB6 |
| 3 | Motors 3+4 — **needle table** (sync) | PC8 | PC9 | PC6 | PD2 / PA15 | PC11 / PA8 |

Drive setup (ZDM‑2HA865 / SS86D class): **2000 pulses/rev**. Cautions:
- **DR+ must tie to the common‑anode +5 V rail** — a floating DR+ shows up as a
  "won't reverse" bug.
- All motors stay **enabled** while powered; a fault/E‑stop frees them for hand‑moving.
- Positive table = **up** toward the panel; negative = **down** to home.
- Per‑bank travel sense is software‑adjustable — see `infeed dir` (§4.4).

### 2.2 Sensors — 11 inputs (PNP, active‑HIGH)

These are the **physical** inputs; their *function* is the assignable role map
(§4). Default role in the right column.

| Sensor | MCU pin | Default logical role |
|---|---|---|
| Laser 1 | PC5 | `panel_presented` (infeed trigger) |
| Laser 2 | PB1 | `panel_near` (start creep) |
| Laser 3 | PB0 | `panel_in_position` (at outfeed) |
| Inductive 4 | PB2 | `cyl1_down`  (stopper retracted) |
| Inductive 5 | PE7 | `cyl1_up`    (stopper raised) |
| Inductive 6 | PE9 | `cyl2_down`  (clamp 2 released) |
| Inductive 7 | PE10 | `cyl2_up`   (clamp 2 engaged) |
| Inductive 8 | PE12 | `cyl3_down` (clamp 3 released) |
| Inductive 9 | PE13 | `cyl3_up`   (clamp 3 engaged) |
| Photo‑electric 10 | PE14 | `table_home` (Motors 3/4 home) |
| Photo‑electric 11 | PE15 | `conveyor_width` (reserved, TBD) |

> The 6 cylinder roles **are now gated** (closed‑loop): after each solenoid the
> cycle waits for the matching `cyl*_up`/`cyl*_down` limit sensor, and **faults if
> it isn't confirmed within 10 s** (production; the confirm waits indefinitely
> while simulating). `conveyor_width` (Motor 2) remains reserved
> (not in the cycle yet).

### 2.3 Solenoid valves — 3× SMC SY3220 (5/2 double‑solenoid, bistable)

Pulse a coil ~60 ms to shift; the valve **holds** with both coils off.
**Coil 1 (A) = actuate, Coil 2 (B) = release.**

| Valve | Cylinder / role | Coil 1 (A) pin | Coil 2 (B) pin |
|---|---|---|---|
| Solenoid 1 | **Stopper gate** (cyl 1): A=raise/lock, B=lower/release | PB12 | PB13 |
| Solenoid 2 | **Clamp #2** (cyl 2): A=clamp, B=release | PB14 | PB15 |
| Solenoid 3 | **Clamp #3** (cyl 3): A=clamp, B=release | PD10 | PD11 |

### 2.4 SMEMA conveyor handshake

| Signal | MCU pin | Dir |
|---|---|---|
| Upstream Board Available | PC1 | IN |
| Upstream Machine Ready | PC2 | OUT |
| Downstream Board Available | PC3 | OUT |
| Downstream Machine Ready | PA1 | IN |

### 2.5 Indicators & emergency stop

- **E‑STOP** = on‑board **user button B1 (PA0)** — stops motion, frees motors,
  de‑energizes solenoids (bistable valves hold), aborts the cycle. Recover with
  `infeed reset` (re‑homes).
- LEDs: **LED3 (PD13)** = Motor 1 moving · **LED4 (PD12)** = table up ·
  **LED5 (PD14)** = table down · LED4+LED5 = table at home.

---

## 3. The operation cycle (summary)

Auto‑homes the table at power‑up, then loops the per‑panel sequence, gated by the
sensors/SMEMA. Five external triggers drive it; each solenoid is **closed‑loop**
(waits for its cylinder limit sensor; alarms after 10 s in production, no timeout while simulating):

1. `panel_presented` ON → raise stopper *(confirm `cyl1_up`)*, convey in
2. `panel_near` ON → creep to seat, clamp Sol2/Sol3 *(confirm `cyl2_up`/`cyl3_up`)*
3. table up → **program PCB (10 s) → "programming success"** → table down to `table_home`
4. release clamps + stopper *(confirm `cyl2_down`/`cyl3_down`/`cyl1_down`)* → convey out
5. `panel_in_position` ON → SMEMA hand‑off downstream → eject → repeat

The event log narrates each step with a traceability label and the **live mapped
sensor**, e.g. `Motor 1 rotating positive 40 rpm until panel near … [panel_near=laser2 PB1]`.

---

## 4. CLI reference

Every command is a sub‑command of `infeed` on the console. `[ ]` = optional
argument; the no‑argument form shows/queries. Map / polarity / direction edits
are **RAM‑only until `infeed cfg save`**.

### 4.0 Command summary

| Command | Purpose |
|---|---|
| `infeed status` | sensors / ALM / PEND / SMEMA / fault / enable + asserted‑now line |
| `infeed watch [secs]` | live‑stream the sensor inputs (default 20 s) |
| `infeed map` | show the role→sensor map (+ duplicate / unused check) |
| `infeed map <role> <sensor>` | bind a role to a sensor |
| `infeed map teach <role>` | bind a role to whichever sensor you actuate next |
| `infeed pol [<sensor> <high\|low>]` | show / set sensor active level (NO/NC) |
| `infeed dir [<1-3> <norm\|inv>]` | show / set per‑bank motor direction |
| `infeed jog <1-3> <+\|-> [rev] [rpm]` | manually jog a bank (default 1 rev @ 20 rpm) |
| `infeed sim <name> <0\|1> \| off` | force an input — logic test without sensors (§6) |
| `infeed cfg save` | persist map + polarity + direction to flash |
| `infeed cfg reset` | restore compiled defaults, then save |
| `infeed cfg dump` | print the config as a portable `INF1:…` token |
| `infeed cfg load <token>` | apply a config token (then `cfg save`) |
| `infeed start` | release the unconfigured safe‑hold / begin |
| `infeed test [on\|off]` | commissioning mode: cylinder confirms wait (no 10 s alarm) |
| `infeed enable <1-3> <on\|off>` | enable or free a drive bank |
| `infeed sol <1-3> <a\|b>` | pulse a valve coil (A=actuate, B=release) |
| `infeed stop` | stop all axes |
| `infeed reset` | clear a latched fault / E‑stop (then re‑homes) |

### 4.1 Status & watch
```
infeed status            Dump sensors / ALM / PEND / SMEMA / fault / enable + 'assert:' line
infeed watch [secs]      Live-stream inputs (default 20 s); '*' marks asserted-at-rest
```
The `assert:` line and `watch`'s `*` flag anything **asserted while at rest** — a
quick way to catch a wrong NO/NC sensor or a miswire. (Note: `table_home` and the
cylinder sensors are *legitimately* asserted when the table/cylinder is parked there.)

### 4.2 Mapping inputs (role ← sensor)
```
infeed map                       Show role -> sensor -> pin table, live state, health check
infeed map <role> <sensor>       Bind a role to a sensor
infeed map teach <role>          "Actuate its sensor now" -> binds whatever input changes
```
- `<role>`: `panel_presented` `panel_near` `panel_in_position` `table_home`
  `conveyor_width` `cyl1_down` `cyl1_up` `cyl2_down` `cyl2_up` `cyl3_down` `cyl3_up`
- `<sensor>`: `laser1` `laser2` `laser3` `inductive4`…`inductive9` `photo10` `photo11`
- `infeed map` (no args) also **warns on duplicate** bindings and lists **unused** sensors.

Teach example (no pin knowledge needed — just walk to the sensor):
```
uart:~$ infeed map teach panel_near
TEACH panel_near: actuate its sensor now (15 s)...
panel_near <- laser2 (PB1)
RAM only — run `infeed cfg save` to persist
```

### 4.3 Sensor polarity (NO / NC)
```
infeed pol                       Show each sensor's active level
infeed pol <sensor> <high|low>   high = active-HIGH (NO) · low = active-LOW (NC)
```
Polarity is **manual** (no auto‑detect by design): a sensor asserted at rest may
be correct (home/clamp) or a NO/NC mismatch — you decide.
```
uart:~$ infeed pol laser1 low
laser1 = active-LOW
```

### 4.4 Motor direction
```
infeed dir                       Show each bank's travel sense
infeed dir <1-3> <norm|inv>      Flip a reversed drive (logical POS stays "forward/up")
```
Use the boot self‑test bob (§4.6) to see which way each bank moves, then `inv`
any bank that runs the wrong way. Persisted with `cfg save`.

### 4.5 Persistence & cloning (flash)
```
infeed cfg save                  Persist map + polarity + direction to flash
infeed cfg reset                 Restore compiled defaults, then save
infeed cfg dump                  Print the config as a portable token (INF1:…)
infeed cfg load <token>          Apply a token (then `cfg save` to keep it)
```
- All map/pol/dir edits are **RAM‑only until `cfg save`**.
- On boot the controller loads the saved config; with none, it uses the defaults
  in this guide **and safe‑holds** (§4.6).
- The **first `cfg save` erases a flash sector (~1–2 s)** — normal; later saves are fast.
- `cfg dump` → copy the `INF1:` token to **clone another machine** in one paste
  (`cfg load <token>` on the next unit, then `cfg save`).
- **Re‑map only while idle / E‑stopped**, never mid‑cycle.

### 4.6 Motion & safety (bench)
```
infeed start                     Release the unconfigured safe-hold / begin
infeed test <on|off>             Commissioning mode: cylinder confirms wait (no 10 s alarm)
infeed enable <1-3> <on|off>     Energize (lock) or free a drive bank — NO motion
infeed jog <1-3> <+|-> [rev] [rpm]   Jog a bank (default 1 rev @ 20 rpm)
infeed sol <1-3> <a|b>           Pulse a valve coil (A=actuate, B=release)
infeed stop                      Stop all axes
infeed reset                     Clear a latched fault / E-stop (then re-homes)
```
- **`enable`** only drives the bank's ENA line: `on` = energized/holding torque
  (ready, but *does not move*), `off` = free for hand‑moving. No speed/direction.
- **`jog`** is the manual move: direction `+`/`-` (logical — `+` = forward/up,
  honoring the `dir` invert), `rev` defaults to **1**, `rpm` to **20**; `rev` may
  be fractional (e.g. `0.25`). It auto‑enables the bank and is refused while that
  bank is moving or a fault is latched. Examples:
  `infeed jog 3 +` (table +1 rev @ 20 rpm) · `infeed jog 1 - 2 40` (conveyor −2 rev @ 40 rpm).
- **Safe‑hold:** a never‑configured unit holds (no motion) at boot until `infeed
  start`. After start it runs the **power‑on self‑test bob** (each bank ±¼ rev,
  banks 2 s apart) then homes. A configured unit (after `cfg save`) auto‑starts.
- **E‑STOP** = on‑board button **B1 (PA0)**; recover with `infeed reset`.

---

## 5. Typical setup / commissioning flow

1. Wire the field I/O per §2 (or to your fixture's actual sensor placement).
2. Power up; open the console (§1). A fresh unit prints `UNCONFIGURED safe-hold`.
3. `infeed start` — watch the 3‑bank self‑test bob; `infeed dir <n> inv` for any
   motor that runs backwards.
4. `infeed watch` — actuate each sensor by hand; confirm it flips (and check its
   rest state isn't unexpectedly asserted).
5. Bind roles: `infeed map teach <role>` (then actuate the sensor) for each, or
   `infeed map <role> <sensor>`. Set `infeed pol <sensor> low` for any NC sensor.
6. `infeed map` — review bindings + health (no duplicate/unused surprises).
7. `infeed cfg save` — persist (also clears the safe‑hold for future boots).
   Optionally `infeed cfg dump` and keep the token to clone other machines.
8. Power‑cycle and `infeed map` to confirm it stuck.

---

## 6. Logic test — simulate the cycle without sensors

Before any real sensors are wired you can walk the **entire 21‑step cycle** from
the console using **`infeed sim`**, which forces an input's state in firmware. The
motors and solenoids **do actuate** (so you also verify those outputs and the
LEDs); only the sensor / SMEMA **inputs** are faked. Forced inputs are RAM‑only
and revert on reboot or with `infeed sim off`.

> ⚠️ Motors will move (or just pulse if not yet connected) and solenoids will
> fire — make sure the mechanics are clear. E‑STOP (B1 / PA0) stops everything;
> recover with `infeed reset`.

Input names: `laser1 laser2 laser3  inductive4…inductive9  photo10 photo11
smema_up smema_down`. With the **default** map the triggers are `laser1` =
panel_presented, `laser2` = panel_near, `photo10` = table_home, `laser3` =
panel_in_position — and the event log always prints the live one as
`[role=sensor pin]`, so just fire whatever it names.

**Walk it line by line — issue each command when the log prints its cue:**

| Step | Log cue | Command(s) |
|---|---|---|
| start | `UNCONFIGURED safe-hold` *(only if unconfigured)* | `infeed start` |
| home | `Homing table … Waiting for [table_home=photo10 …]` | `infeed sim photo10 1` → `infeed sim photo10 0` |
| #1 | `#1 idle … Board Available` | `infeed sim smema_up 1` → `infeed sim smema_up 0` |
| #3 | `Waiting for [panel_presented=laser1 …]` | `infeed sim laser1 1` → `infeed sim laser1 0` |
| #4 | `Raise stopper gate …` | `infeed sim inductive5 1` *(cyl1_up)* |
| #6 | `Waiting for [panel_near=laser2 …]` | `infeed sim laser2 1` → `infeed sim laser2 0` |
| #8 | `Clamp panel 2 …` | `infeed sim inductive7 1` *(cyl2_up)* |
| #9 | `Clamp panel 3 …` | `infeed sim inductive9 1` *(cyl3_up)* |
| #12 | `Waiting for [table_home=photo10 …]` | `infeed sim photo10 1` → `infeed sim photo10 0` |
| #13 | `Release clamp 2 …` | `infeed sim inductive7 0` → `infeed sim inductive6 1` *(cyl2_down)* |
| #14 | `Release clamp 3 …` | `infeed sim inductive9 0` → `infeed sim inductive8 1` *(cyl3_down)* |
| #15 | `Lower stopper gate …` | `infeed sim inductive5 0` → `infeed sim inductive4 1` *(cyl1_down)* |
| #17 | `Waiting for [panel_in_position=laser3 …]` | `infeed sim laser3 1` → `infeed sim laser3 0` |
| #20 | `Waiting for downstream 'Ready'` | `infeed sim smema_down 1` → `infeed sim smema_down 0` |

> ℹ️ **Cylinder confirms — no timeout during simulation.** Steps #4/#8/#9/#13/#14/#15
> pulse a solenoid and then **wait for the matching cylinder limit sensor**
> (`cyl*_up`/`cyl*_down` = Inductive 4–9). While a sim is active (`infeed sim` has
> forced any input) the confirm **waits indefinitely** — take your time; it logs
> `… SIM active, waiting (no timeout) …`. Fire the `inductive*` line for the
> position the cylinder just moved to (set the new one, clear the opposite). In
> **production** (no sim) these confirms **alarm after 10 s** if the cylinder
> doesn't reach position (`CYL_CONFIRM_MS`) — a stuck‑cylinder / no‑air / dead‑sensor
> fault; recover with `infeed reset`.

After #20 the conveyor ejects (~60 s) and the log prints `==== Cycle 1 done ====`,
then returns to `#1 idle`. Repeat from the `smema_up` line for another panel, then
`infeed sim off` to return to live inputs.

**Timing:** between cues the controller runs real moves — table up ≈ 42 s, table
down ≈ 30 s, eject ≈ 60 s — so a full simulated cycle takes a few minutes. The
log's `Waiting for …` lines tell you exactly when the next `sim` is due.

**Watch:** LED3 = conveyor moving · LED4/LED5 = table up/down (both = at home);
solenoid steps `Sol1.C1 (PB12)` … fire in order. `infeed status` shows
`SIM ACTIVE` while any input is forced.

**Full command sequence (one line per cue, in order):**
```
infeed start
infeed sim photo10 1
infeed sim photo10 0
infeed sim smema_up 1
infeed sim smema_up 0
infeed sim laser1 1
infeed sim laser1 0
infeed sim inductive5 1
infeed sim laser2 1
infeed sim laser2 0
infeed sim inductive7 1
infeed sim inductive9 1
infeed sim photo10 1
infeed sim photo10 0
infeed sim inductive7 0
infeed sim inductive6 1
infeed sim inductive9 0
infeed sim inductive8 1
infeed sim inductive5 0
infeed sim inductive4 1
infeed sim laser3 1
infeed sim laser3 0
infeed sim smema_down 1
infeed sim smema_down 0
infeed sim off
```

> Tip: to dry‑run **only the logic** with no mechanical motion, run the test with
> the drives unpowered/disconnected — the step generator still completes each move
> in real time, so the sequence advances exactly the same; you just watch the log
> + LEDs + solenoids.
