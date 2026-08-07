"""
Serial console and configuration tool for the motion controller.

Usage:
  python send_config.py <COM_PORT> [config_file] [--delay=MS] [--port=TCP_PORT]
  python send_config.py <COM_PORT> --dump [output_file]
  python send_config.py --connect [HOST:]PORT [config_file] [--delay=MS]

Modes:
  serve (default):   Opens COM port, starts TCP server for remote commands.
                     All serial traffic is displayed. Keyboard input accepted.
  --dump:            Reads config from board and saves to file.
  --connect:         Connects to a running serve instance via TCP and sends
                     commands (from file or stdin). Does NOT open COM port.

Examples:
  python send_config.py COM19                              # Console + TCP on 5555
  python send_config.py COM19 --port=6000                  # Console + TCP on 6000
  python send_config.py COM19 config/default_config.txt    # Send config, then console
  python send_config.py COM19 --dump                       # Dump to default_config.txt
  python send_config.py COM19 --dump my_config.txt         # Dump to custom file
  python send_config.py --connect 5555 config/default.txt  # Send via TCP
  python send_config.py --connect 5555                     # Interactive via TCP

Console:
  - Type commands and press Enter to send
  - Ctrl+C to exit
"""

import sys
import time
import threading
import socket
import os
import re
import msvcrt

import serial

DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "default_config.txt")
DEFAULT_BAUD = 115200
DEFAULT_DELAY_MS = 50
DEFAULT_TCP_PORT = 5555

# Global flag to pause reader_thread during setup mode
_reader_paused = [False]


def reader_thread(ser, running, tcp_clients, tcp_lock):
    """Background thread that prints incoming serial data and forwards to TCP clients."""
    while running[0]:
        try:
            if _reader_paused[0]:
                time.sleep(0.05)
                continue
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                text = data.decode(errors="replace")
                sys.stdout.write(text)
                sys.stdout.flush()
                with tcp_lock:
                    dead = []
                    for client in tcp_clients:
                        try:
                            client.sendall(data)
                        except (OSError, BrokenPipeError):
                            dead.append(client)
                    for c in dead:
                        tcp_clients.remove(c)
            else:
                time.sleep(0.02)
        except (OSError, serial.SerialException):
            break


def tcp_client_handler(client_sock, addr, ser, running, tcp_clients, tcp_lock):
    """Handle a single TCP client: forward its lines to serial."""
    print(f"\n[TCP] Client connected: {addr}")
    with tcp_lock:
        tcp_clients.append(client_sock)
    buf = b""
    try:
        while running[0]:
            try:
                data = client_sock.recv(1024)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if line:
                        ser.write(line + b"\r\n")
                        time.sleep(0.05)
            except (OSError, ConnectionResetError):
                break
    finally:
        with tcp_lock:
            if client_sock in tcp_clients:
                tcp_clients.remove(client_sock)
        client_sock.close()
        print(f"\n[TCP] Client disconnected: {addr}")


def tcp_server_thread(ser, running, tcp_port, tcp_clients, tcp_lock):
    """TCP server that accepts connections and spawns handlers."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", tcp_port))
    srv.listen(4)
    srv.settimeout(1.0)
    print(f"[TCP] Listening on port {tcp_port}")
    while running[0]:
        try:
            client, addr = srv.accept()
            t = threading.Thread(target=tcp_client_handler,
                                 args=(client, addr, ser, running, tcp_clients, tcp_lock),
                                 daemon=True)
            t.start()
        except socket.timeout:
            continue
        except OSError:
            break
    srv.close()


def send_config(ser, config_file, delay_ms):
    """Send config file lines to the serial port."""
    with open(config_file, "r") as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith("#")]

    print(f"\n--- Sending {len(lines)} commands (delay={delay_ms}ms) ---")
    for line in lines:
        ser.write((line + "\r\n").encode())
        time.sleep(delay_ms / 1000.0)
        time.sleep(0.05)
    print(f"--- All {len(lines)} commands sent ---\n")


def dump_config(ser, output_file):
    """Send 'cfg dump' to board, capture output, save as loadable config file."""
    ser.reset_input_buffer()
    ser.write(b"cfg dump\r\n")

    lines = []
    deadline = time.time() + 5.0
    capturing = False

    while time.time() < deadline:
        if ser.in_waiting:
            raw = ser.readline().decode(errors="replace").strip()
            clean = re.sub(r'\x1b\[[0-9;]*m', '', raw)
            clean = clean.replace('\ufffd', '')
            if clean == "#CFG_V1":
                capturing = True
                continue
            if clean == "#END":
                break
            if capturing and "=" in clean:
                lines.append(clean)
            deadline = time.time() + 2.0
        else:
            time.sleep(0.02)

    if not lines:
        print("ERROR: No config data received from board.")
        return False

    with open(output_file, "w", encoding="utf-8") as f:
        f.write("cfg unlock factory\n")
        for line in lines:
            f.write(f"cfg load {line}\n")
        f.write("cfg save\n")

    print(f"--- Dumped {len(lines)} parameters to {output_file} ---")
    return True


def console(ser):
    """Interactive console — read user input, send to serial."""
    print("--- Console active (type 'exit' or 'quit' to close, 'setup' for config menu) ---")
    while True:
        try:
            line = input()
            if line.strip().lower() in ("exit", "quit"):
                print("\n--- Disconnected ---")
                break
            if line.strip().lower() == "setup":
                setup_menu(ser)
                print("--- Console active (type 'exit' or 'quit' to close, 'setup' for config menu) ---")
                continue
            ser.write((line + "\r\n").encode())
        except KeyboardInterrupt:
            print()
            continue
        except EOFError:
            print("\n--- Disconnected ---")
            break


# ---------------------------------------------------------------------------
# Interactive Setup Menu
# ---------------------------------------------------------------------------

def send_and_capture(ser, cmd, timeout=2.0):
    """Send a command and capture response lines until silence."""
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode())
    lines = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            raw = ser.readline().decode(errors="replace").strip()
            clean = re.sub(r'\x1b\[[0-9;]*m', '', raw)
            clean = clean.replace('\ufffd', '')
            if clean and clean != cmd:
                lines.append(clean)
            deadline = time.time() + 0.3
        else:
            time.sleep(0.02)
    return lines


def _clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def _get_key():
    """Read a single key press (Windows). Returns the character or special key name."""
    ch = msvcrt.getch()
    if ch in (b'\x00', b'\xe0'):
        ch2 = msvcrt.getch()
        if ch2 == b'K':
            return '<'
        elif ch2 == b'M':
            return '>'
        elif ch2 == b'H':
            return 'up'
        elif ch2 == b'P':
            return 'down'
        return None
    return ch.decode(errors="replace")


def _print_header(title, path=None):
    print(f"\n{'='*50}")
    if path:
        print(f"  {path}")
        print(f"{'─'*50}")
    print(f"  {title}")
    print(f"{'='*50}\n")


# --- Parameter Definitions ---

CONVEYOR_PARAMS = [
    ("Home Speed",      "cfg set conveyor home_rpm {v}",     1,    "mm/s"),
    ("Accel Start",     "cfg set conveyor accel_start {v}",  1,    "mm/s"),
    ("Accel Rate",      "cfg set conveyor accel_rate {v}",   5,    "mm/s2"),
    ("Belt mm/rev",     "cfg set conveyor mm_per_rev {v}",   1.0,  "mm"),
]

HEAD_PARAMS = [
    ("Home Speed",      "cfg set head home_rpm {v}",        1,    "mm/s"),
    ("Accel Start",     "cfg set head accel_start {v}",     1,    "mm/s"),
    ("Accel Rate",      "cfg set head accel_rate {v}",      5,    "mm/s2"),
    ("mm/rev",          "cfg set head mm_per_rev {v}",      0.5,  "mm"),
    ("Safe Position",   "cfg set head safe_pos {v}",        100,  "steps"),
    ("Guides Clear",    "cfg set head guides_clear {v}",    0.5,  "mm"),
    ("Pins Touch",      "cfg set head pins_touch {v}",      0.5,  "mm"),
    ("Pins Contact",    "cfg set head pins_contact {v}",    0.5,  "mm"),
    ("Soft Limit Min",  "cfg set head limit_min {v}",       1.0,  "mm"),
    ("Soft Limit Max",  "cfg set head limit_max {v}",       1.0,  "mm"),
]

WIDTH_PARAMS = [
    ("Home Speed",      "cfg set width home_rpm {v}",        1,    "mm/s"),
    ("Accel Start",     "cfg set width accel_start {v}",     1,    "mm/s"),
    ("Accel Rate",      "cfg set width accel_rate {v}",      5,    "mm/s2"),
    ("mm/rev",          "cfg set width mm_per_rev {v}",      0.5,  "mm"),
    ("Safe Position",   "cfg set width safe_pos {v}",        100,  "steps"),
    ("Soft Limit Min",  "cfg set width limit_min {v}",       1.0,  "mm"),
    ("Soft Limit Max",  "cfg set width limit_max {v}",       1.0,  "mm"),
]

SENSOR_NAMES = [
    "laser1", "laser2", "laser3", "head_home",
    "ind4", "ind5", "ind6", "ind7", "ind8", "ind9"
]

CYLINDER_NAMES = ["stopper", "rfid", "locker"]

NODES = [
    ("Conveyor",   CONVEYOR_PARAMS),
    ("Head",       HEAD_PARAMS),
    ("Width",      WIDTH_PARAMS),
    ("Sensors",    None),
    ("Cylinders",  None),
    ("SMEMA",      None),
]


def _parse_cfg_show(lines):
    """Parse 'cfg show' output into a dict of param_name -> value_string."""
    params = {}
    for line in lines:
        m = re.match(r'\s*(\S+)\s*=\s*(.+)', line)
        if m:
            params[m.group(1).strip()] = m.group(2).strip()
    return params


def _get_current_value(cfg_data, param_name, cmd_template, unit):
    """Try to extract current value for a parameter from cfg show data."""
    key_map = {
        "cfg set conveyor home_rpm": "home_rpm",
        "cfg set conveyor accel_start": "accel_start_rpm",
        "cfg set conveyor accel_rate": "accel_rpm_s",
        "cfg set conveyor mm_per_rev": "belt_mm_per_rev",
        "cfg set head home_rpm": "home_rpm",
        "cfg set head accel_start": "accel_start_rpm",
        "cfg set head accel_rate": "accel_rpm_s",
        "cfg set head mm_per_rev": "mm_per_rev",
        "cfg set head safe_pos": "safe_position",
        "cfg set head guides_clear": "guides_clear",
        "cfg set head pins_touch": "pins_touch",
        "cfg set head pins_contact": "pins_contact",
        "cfg set head limit_min": "soft_limit_min",
        "cfg set head limit_max": "soft_limit_max",
        "cfg set width home_rpm": "home_rpm",
        "cfg set width accel_start": "accel_start_rpm",
        "cfg set width accel_rate": "accel_rpm_s",
        "cfg set width mm_per_rev": "width_mm_per_rev",
        "cfg set width safe_pos": "safe_position",
        "cfg set width limit_min": "soft_limit_min",
        "cfg set width limit_max": "soft_limit_max",
    }
    base_cmd = cmd_template.split("{v}")[0].strip()
    field = key_map.get(base_cmd, "")
    if field and field in cfg_data:
        val = cfg_data[field]
        num = re.match(r'[-\d.]+', val)
        if not num:
            return val
        raw = float(num.group(0))
        # Convert RPM to mm/s if the unit asks for it
        if unit in ("mm/s", "mm/s2"):
            motor = _motor_from_cmd(base_cmd)
            mpr = _get_mm_per_rev(cfg_data, motor)
            if mpr > 0:
                converted = raw * mpr / 60.0
                return f"{converted:.1f}"
            return f"N/A"
        # For steps-based limits on linear axes, show as mm
        if unit == "mm" and field in ("soft_limit_min", "soft_limit_max"):
            mm_match = re.search(r'\(([-\d.]+)\s*mm\)', val)
            if mm_match:
                return mm_match.group(1)
        if '.' in num.group(0):
            return num.group(0)
        return str(int(raw))
    return "?"


def _get_mm_per_rev(cfg_data, motor_name):
    """Get mm_per_rev for a motor from cfg_data (read from cfg show)."""
    if motor_name == "head":
        val = cfg_data.get("mm_per_rev", "0")
    elif motor_name == "width":
        val = cfg_data.get("width_mm_per_rev", "0")
    elif motor_name == "conveyor":
        val = cfg_data.get("belt_mm_per_rev", "0")
    else:
        return 0
    m = re.match(r'[-\d.]+', str(val))
    return float(m.group(0)) if m else 0


def _motor_from_cmd(base_cmd):
    """Extract motor name from a cfg set command prefix."""
    parts = base_cmd.split()
    if len(parts) >= 3:
        return parts[2]
    return ""


def _read_current_values(ser, node_idx):
    """Read current config from the board for a given node."""
    lines = send_and_capture(ser, "cfg show", timeout=3.0)
    sections = {}
    current_section = None
    for line in lines:
        if line.startswith("["):
            current_section = line.strip("[]").strip()
            sections[current_section] = {}
        elif current_section:
            m = re.match(r'\s*(\S+)\s*=\s*(.+)', line)
            if m:
                sections[current_section][m.group(1)] = m.group(2).strip()
            accel_m = re.search(r'start_rpm=(\d+)\s+accel_rpm_s=(\d+)', line)
            if accel_m:
                sections[current_section]["accel_start_rpm"] = accel_m.group(1)
                sections[current_section]["accel_rpm_s"] = accel_m.group(2)

    motor_map = {0: "1 conveyor", 1: "3 head", 2: "2 width"}
    section_key = motor_map.get(node_idx, "")

    result = {}
    if section_key in sections:
        result.update(sections[section_key])
    if "head positions" in sections and node_idx == 1:
        result.update(sections["head positions"])
    # Merge Conveyor/Head linear config sections
    if node_idx in (0, 2) and "conveyor" in sections:
        result.update(sections["conveyor"])
    if node_idx == 1 and "head" in sections:
        result.update(sections["head"])
    # For width, also grab width_mm_per_rev from conveyor section
    if node_idx == 2 and "conveyor" in sections:
        result.update(sections["conveyor"])
    return result


def _show_motor_params(ser, node_idx, params, level_name):
    """Display motor parameters and allow editing."""
    while True:
        cfg_data = _read_current_values(ser, node_idx)
        _clear_screen()
        node_name = NODES[node_idx][0]
        _print_header(f"{node_name} Parameters", f"{level_name} / {node_name}")

        for i, (name, cmd, step, unit) in enumerate(params):
            val = _get_current_value(cfg_data, name, cmd, unit)
            print(f"  {i+1:2d}. {name:<20s} = {val} {unit}")

        print(f"\n  [1-{len(params)}] Select parameter")
        print("  [s] Save to flash")
        print("  [q] Back\n")

        key = _get_key()
        if key in ('q', 'Q', '\x1b'):
            return
        if key in ('s', 'S'):
            send_and_capture(ser, "cfg save")
            print("  >> Saved to flash!")
            time.sleep(1)
            continue

        if key and key.isdigit():
            idx = int(key) - 1
            if 0 <= idx < len(params):
                _adjust_param(ser, params[idx], node_idx, level_name)


def _adjust_param(ser, param_def, node_idx, level_name):
    """Adjust a single parameter: direct value or < / > stepping."""
    name, cmd_template, step, unit = param_def
    base_cmd = cmd_template.split("{v}")[0].strip()
    motor = _motor_from_cmd(base_cmd)
    node_name = NODES[node_idx][0]

    while True:
        cfg_data = _read_current_values(ser, node_idx)
        mpr = _get_mm_per_rev(cfg_data, motor)
        is_linear = unit in ("mm/s", "mm/s2") and mpr > 0
        current = _get_current_value(cfg_data, name, cmd_template, unit)

        _clear_screen()
        _print_header(f"Adjust: {name}", f"{level_name} / {node_name} / {name}")
        print(f"  Current value: {current} {unit}")
        print(f"  Step size:     {step} {unit}")
        print()
        print("  [<] / [Left]   Decrease by step")
        print("  [>] / [Right]  Increase by step")
        print("  [v]            Enter value directly")
        print("  [q] / [Esc]    Back\n")

        key = _get_key()
        if key in ('q', 'Q', '\x1b'):
            return

        if key == '<':
            try:
                new_display = float(current) - step
                hw_val = new_display * 60.0 / mpr if is_linear else new_display
                if is_linear:
                    hw_val = int(round(hw_val))
                elif step == int(step) and '.' not in str(current):
                    hw_val = int(hw_val)
                cmd = cmd_template.format(v=hw_val)
                send_and_capture(ser, cmd)
            except (ValueError, TypeError):
                pass

        elif key == '>':
            try:
                new_display = float(current) + step
                hw_val = new_display * 60.0 / mpr if is_linear else new_display
                if is_linear:
                    hw_val = int(round(hw_val))
                elif step == int(step) and '.' not in str(current):
                    hw_val = int(hw_val)
                cmd = cmd_template.format(v=hw_val)
                send_and_capture(ser, cmd)
            except (ValueError, TypeError):
                pass

        elif key == 'v' or key == 'V':
            print(f"  Enter new value ({unit}): ", end="", flush=True)
            val_str = ""
            while True:
                ch = _get_key()
                if ch == '\r' or ch == '\n':
                    break
                if ch == '\x1b':
                    val_str = ""
                    break
                if ch == '\x08':
                    if val_str:
                        val_str = val_str[:-1]
                        print("\b \b", end="", flush=True)
                    continue
                if ch and (ch.isdigit() or ch in '.-'):
                    val_str += ch
                    print(ch, end="", flush=True)
            print()
            if val_str:
                try:
                    val = float(val_str)
                    hw_val = val * 60.0 / mpr if is_linear else val
                    if is_linear:
                        hw_val = int(round(hw_val))
                    elif step == int(step) and '.' not in val_str:
                        hw_val = int(hw_val)
                    cmd = cmd_template.format(v=hw_val)
                    send_and_capture(ser, cmd)
                except ValueError:
                    print("  Invalid value!")
                    time.sleep(1)


def _show_sensors(ser, level_name):
    """Show and configure sensor polarities."""
    while True:
        lines = send_and_capture(ser, "cfg sensor", timeout=2.0)
        _clear_screen()
        _print_header("Sensor Polarity", f"{level_name} / Sensors")

        sensor_states = {}
        for line in lines:
            for i, sname in enumerate(SENSOR_NAMES):
                if sname in line:
                    val = "HIGH" if "active_high=1" in line or "HIGH" in line.upper() else "LOW"
                    if "1" in line.split("=")[-1] if "=" in line else "":
                        val = "active_high"
                    sensor_states[sname] = line
                    break

        parsed = send_and_capture(ser, "diag sensors", timeout=2.0)
        for i, sname in enumerate(SENSOR_NAMES):
            state = "?"
            for line in lines:
                if sname in line:
                    state = line.strip()
                    break
            print(f"  {i+1:2d}. {state}")

        print(f"\n  [1-{len(SENSOR_NAMES)}] Toggle polarity")
        print("  [q] Back\n")

        key = _get_key()
        if key in ('q', 'Q', '\x1b'):
            return
        if key and key.isdigit():
            idx = int(key) - 1
            if 0 <= idx < len(SENSOR_NAMES):
                name = SENSOR_NAMES[idx]
                current_lines = send_and_capture(ser, f"cfg sensor {name}", timeout=1.0)
                is_high = any("1" in l.split("=")[-1] for l in current_lines if "=" in l)
                new_val = 0 if is_high else 1
                send_and_capture(ser, f"cfg sensor {name} {new_val}")


def _show_cylinders(ser, level_name):
    """Show and configure cylinder settings."""
    while True:
        _clear_screen()
        _print_header("Cylinder Configuration", f"{level_name} / Cylinders")

        lines = send_and_capture(ser, "cfg cylinder read", timeout=2.0)
        for i, line in enumerate(lines):
            print(f"  {line}")

        print(f"\n  [f] Fire test (select cylinder)")
        print("  [n] Rename cylinder")
        print("  [q] Back\n")

        key = _get_key()
        if key in ('q', 'Q', '\x1b'):
            return
        if key == 'f':
            print("  Cylinder (1=stopper, 2=rfid, 3=locker): ", end="", flush=True)
            k = _get_key()
            if k in ('1', '2', '3'):
                cyl = CYLINDER_NAMES[int(k)-1]
                print(f"{cyl}")
                print("  Position (a/b): ", end="", flush=True)
                p = _get_key()
                if p in ('a', 'b'):
                    print(p)
                    send_and_capture(ser, f"manual cylinder {cyl} {p}")
                    time.sleep(1)


def _show_smema(ser, level_name):
    """SMEMA diagnostics and manual control."""
    while True:
        lines = send_and_capture(ser, "manual smema", timeout=1.5)
        _clear_screen()
        _print_header("SMEMA Control", f"{level_name} / SMEMA")

        for line in lines:
            if "Current state" in line or "Up BA" in line or "Down MR" in line:
                print(f"  {line}")

        print()
        print("  [1] MR OUT = HIGH     [2] MR OUT = LOW")
        print("  [3] BA OUT = HIGH     [4] BA OUT = LOW")
        print("  [5] BA FAIL = HIGH    [6] BA FAIL = LOW")
        print("  [0] All OFF")
        print("  [q] Back\n")

        key = _get_key()
        if key in ('q', 'Q', '\x1b'):
            send_and_capture(ser, "manual smema off")
            return
        cmds = {
            '1': "manual smema mr_out 1",
            '2': "manual smema mr_out 0",
            '3': "manual smema ba_out 1",
            '4': "manual smema ba_out 0",
            '5': "manual smema ba_fail_out 1",
            '6': "manual smema ba_fail_out 0",
            '0': "manual smema off",
        }
        if key in cmds:
            send_and_capture(ser, cmds[key])


def setup_menu(ser):
    """Main setup menu entry point."""
    _reader_paused[0] = True
    time.sleep(0.1)
    ser.reset_input_buffer()

    try:
        _setup_menu_inner(ser)
    finally:
        _reader_paused[0] = False


def _setup_menu_inner(ser):
    """Internal setup menu logic."""
    # Level selection
    _clear_screen()
    _print_header("Setup Mode")
    print("  Access Level:")
    print("  1. Engineering (proc_eng)")
    print("  2. Factory")
    print("  q. Cancel\n")

    key = _get_key()
    if key in ('q', 'Q', '\x1b'):
        return

    if key == '1':
        send_and_capture(ser, "cfg unlock proc_eng")
        level_name = "Engineering"
    elif key == '2':
        send_and_capture(ser, "cfg unlock factory")
        level_name = "Factory"
    else:
        return

    # Node selection loop
    while True:
        _clear_screen()
        _print_header("Select Node", level_name)
        for i, (name, _) in enumerate(NODES):
            print(f"  {i+1}. {name}")
        print("\n  [s] Save to flash")
        print("  [q] Exit setup\n")

        key = _get_key()
        if key in ('q', 'Q', '\x1b'):
            send_and_capture(ser, "cfg lock")
            return
        if key in ('s', 'S'):
            send_and_capture(ser, "cfg save")
            print("  >> Saved to flash!")
            time.sleep(1)
            continue

        if key and key.isdigit():
            idx = int(key) - 1
            if idx == 0:
                _show_motor_params(ser, 0, CONVEYOR_PARAMS, level_name)
            elif idx == 1:
                _show_motor_params(ser, 1, HEAD_PARAMS, level_name)
            elif idx == 2:
                _show_motor_params(ser, 2, WIDTH_PARAMS, level_name)
            elif idx == 3:
                _show_sensors(ser, level_name)
            elif idx == 4:
                _show_cylinders(ser, level_name)
            elif idx == 5:
                _show_smema(ser, level_name)


def connect_mode(host, port, config_file, delay_ms):
    """Connect to a running serve instance via TCP."""
    print(f"Connecting to {host}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("Connected.")

    # Reader thread for responses
    running = [True]

    def tcp_reader():
        while running[0]:
            try:
                data = sock.recv(4096)
                if not data:
                    break
                sys.stdout.write(data.decode(errors="replace"))
                sys.stdout.flush()
            except (OSError, ConnectionResetError):
                break

    reader = threading.Thread(target=tcp_reader, daemon=True)
    reader.start()

    try:
        if config_file:
            with open(config_file, "r") as f:
                lines = [l.strip() for l in f if l.strip() and not l.startswith("#")]
            print(f"\n--- Sending {len(lines)} commands (delay={delay_ms}ms) ---")
            for line in lines:
                sock.sendall((line + "\n").encode())
                time.sleep(delay_ms / 1000.0)
            print(f"--- All {len(lines)} commands sent ---\n")
            time.sleep(1.0)
        else:
            print("--- Interactive mode (Ctrl+C to exit) ---")
            while True:
                line = input()
                sock.sendall((line + "\n").encode())
    except (KeyboardInterrupt, EOFError):
        print("\n--- Disconnected ---")
    finally:
        running[0] = False
        sock.close()


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    # Check for --connect mode
    if sys.argv[1] == "--connect":
        if len(sys.argv) < 3:
            print("usage: send_config.py --connect [host:]port [config_file] [--delay=MS]")
            sys.exit(1)
        target = sys.argv[2]
        if ":" in target:
            host, port = target.rsplit(":", 1)
            port = int(port)
        else:
            host = "127.0.0.1"
            port = int(target)
        config_file = None
        delay_ms = DEFAULT_DELAY_MS
        for arg in sys.argv[3:]:
            if arg.startswith("--delay="):
                delay_ms = int(arg.split("=")[1])
            else:
                config_file = arg
        connect_mode(host, port, config_file, delay_ms)
        return

    port = sys.argv[1]
    config_file = None
    delay_ms = DEFAULT_DELAY_MS
    dump_mode = False
    dump_output = DEFAULT_CONFIG
    tcp_port = DEFAULT_TCP_PORT

    args = sys.argv[2:]
    i = 0
    while i < len(args):
        if args[i] == "--dump":
            dump_mode = True
            if i + 1 < len(args) and not args[i + 1].startswith("--"):
                dump_output = args[i + 1]
                i += 1
        elif args[i].startswith("--delay="):
            delay_ms = int(args[i].split("=")[1])
        elif args[i].startswith("--port="):
            tcp_port = int(args[i].split("=")[1])
        else:
            config_file = args[i]
        i += 1

    if config_file and not os.path.exists(config_file):
        print(f"ERROR: Config file not found: {config_file}")
        sys.exit(1)

    print(f"Opening {port} at {DEFAULT_BAUD} baud...")
    ser = serial.Serial(port, DEFAULT_BAUD, timeout=0.1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    if dump_mode:
        dump_config(ser, dump_output)
        ser.close()
        return

    running = [True]
    tcp_clients = []
    tcp_lock = threading.Lock()

    reader = threading.Thread(target=reader_thread,
                              args=(ser, running, tcp_clients, tcp_lock), daemon=True)
    reader.start()

    tcp_srv = threading.Thread(target=tcp_server_thread,
                               args=(ser, running, tcp_port, tcp_clients, tcp_lock), daemon=True)
    tcp_srv.start()

    try:
        if config_file:
            send_config(ser, config_file, delay_ms)

        console(ser)
    finally:
        running[0] = False
        ser.close()


if __name__ == "__main__":
    main()
