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


_controller_ready = [True]
_controller_rebooted = [False]


def reader_thread(ser_holder, running, tcp_clients, tcp_lock):
    """Background thread that prints incoming serial data and forwards to TCP clients."""
    while running[0]:
        try:
            ser = ser_holder[0]
            if ser is None or not ser.is_open:
                time.sleep(0.1)
                continue
            if _reader_paused[0]:
                time.sleep(0.05)
                continue
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                if b"Booting" in data or b"booting" in data:
                    _controller_ready[0] = False
                    _controller_rebooted[0] = True
                    sys.stdout.write("\n[Controller] Reset detected\n")
                    sys.stdout.flush()
                if b"init complete" in data:
                    _controller_ready[0] = True
                    sys.stdout.write("[Controller] Ready\n")
                    sys.stdout.flush()
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
            time.sleep(0.1)


def tcp_client_handler(client_sock, addr, ser_holder, running, tcp_clients, tcp_lock):
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
                        ser = ser_holder[0]
                        if ser and ser.is_open:
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


def tcp_server_thread(ser_holder, running, tcp_port, tcp_clients, tcp_lock):
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
                                 args=(client, addr, ser_holder, running, tcp_clients, tcp_lock),
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


def console_loop(ser_holder, running):
    """Interactive console — read user input, send to serial."""
    print("--- Console active (type 'exit' or 'quit' to close, 'setup' for config menu) ---")
    while running[0]:
        try:
            line = input()
            if line.strip().lower() in ("exit", "quit"):
                print("\n--- Disconnected ---")
                break
            if line.strip().lower() == "setup":
                ser = ser_holder[0]
                if ser and ser.is_open:
                    setup_menu(ser)
                else:
                    print("[Serial] Not connected")
                print("--- Console active (type 'exit' or 'quit' to close, 'setup' for config menu) ---")
                continue
            ser = ser_holder[0]
            if ser and ser.is_open:
                ser.write((line + "\r\n").encode())
            else:
                print("[Serial] Not connected — waiting for reconnect...")
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
    try:
        if not ser or not ser.is_open:
            return []
        if not _controller_ready[0]:
            return []
        ser.reset_input_buffer()
        ser.write((cmd + "\r\n").encode())
    except (OSError, serial.SerialException):
        return []
    lines = []
    deadline = time.time() + timeout
    try:
        while time.time() < deadline:
            if ser.in_waiting:
                raw = ser.readline().decode(errors="replace").strip()
                clean = re.sub(r'\x1b\[[0-9;]*m', '', raw)
                clean = clean.replace('\ufffd', '')
                if "Booting Zephyr" in clean or "*** Booting" in clean:
                    _controller_ready[0] = False
                    _controller_rebooted[0] = True
                elif "machine init complete" in clean and not _controller_ready[0]:
                    _controller_ready[0] = True
                if clean and clean != cmd:
                    lines.append(clean)
                deadline = time.time() + 0.3
            else:
                time.sleep(0.02)
    except (OSError, serial.SerialException):
        pass
    return lines


def _clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def _check_serial_for_boot(ser):
    """Read available serial data and check for boot patterns. Non-blocking."""
    try:
        n = ser.in_waiting
        if n > 0:
            data = ser.read(n)
            if b"Booting" in data or b"booting" in data:
                _controller_ready[0] = False
                _controller_rebooted[0] = True
            if b"init complete" in data:
                _controller_ready[0] = True
    except (OSError, serial.SerialException):
        pass


def _get_key(ser=None):
    """Read a single key press (Windows). Checks serial for boot messages while waiting."""
    while True:
        if msvcrt.kbhit():
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
        if ser:
            _check_serial_for_boot(ser)
            if _controller_rebooted[0]:
                return None
        time.sleep(0.02)


def _print_header(title, path=None):
    print(f"\n{'='*50}")
    if path:
        print(f"  {path}")
        print(f"{'─'*50}")
    print(f"  {title}")
    print(f"{'='*50}\n")


# --- Parameter Definitions ---

CONVEYOR_PARAMS = [
    ("Convey Speed",      "cfg set conveyor convey_speed {v}",      5,    "mm/s"),
    ("Creep Speed",       "cfg set conveyor creep_speed {v}",       1,    "mm/s"),
    ("Convey Out Speed",  "cfg set conveyor convey_out_speed {v}",  5,    "mm/s"),
    ("Eject Speed",       "cfg set conveyor eject_speed {v}",       1,    "mm/s"),
    ("Creep Distance",    "cfg set conveyor creep_distance {v}",    5.0,  "mm"),
    ("Belt mm/rev",       "cfg set conveyor mm_per_rev {v}",        1.0,  "mm"),
    ("Width mm/rev",      "cfg set width mm_per_rev {v}",           0.5,  "mm"),
]

HEAD_PARAMS = [
    ("Fast Speed",      "cfg set head fast_speed {v}",        1,    "mm/s"),
    ("Slow Speed",      "cfg set head slow_speed {v}",        0.5,  "mm/s"),
    ("mm/rev",          "cfg set head mm_per_rev {v}",        0.5,  "mm"),
    ("Guides Clear",    "cfg set head guides_clear {v}",      0.5,  "mm"),
    ("Pins Touch",      "cfg set head pins_touch {v}",        0.5,  "mm"),
    ("Pins Contact",    "cfg set head pins_contact {v}",      0.5,  "mm"),
    ("Soft Limit Min",  "cfg set head limit_min {v}",         1.0,  "mm"),
    ("Soft Limit Max",  "cfg set head limit_max {v}",         1.0,  "mm"),
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
        "cfg set conveyor convey_speed": "convey_speed",
        "cfg set conveyor creep_speed": "creep_speed",
        "cfg set conveyor convey_out_speed": "convey_out_speed",
        "cfg set conveyor eject_speed": "eject_speed",
        "cfg set conveyor creep_distance": "creep_distance",
        "cfg set conveyor mm_per_rev": "belt_mm_per_rev",
        "cfg set width mm_per_rev": "width_mm_per_rev",
        "cfg set head fast_speed": "fast_speed",
        "cfg set head slow_speed": "slow_speed",
        "cfg set head mm_per_rev": "mm_per_rev",
        "cfg set head guides_clear": "guides_clear",
        "cfg set head pins_touch": "pins_touch",
        "cfg set head pins_contact": "pins_contact",
        "cfg set head limit_min": "soft_limit_min",
        "cfg set head limit_max": "soft_limit_max",
        "cfg set width home_rpm": "home_rpm",
        "cfg set width accel_start": "accel_start_rpm",
        "cfg set width accel_rate": "accel_rpm_s",
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
        # Width motor still uses RPM in firmware, convert to mm/s
        if unit in ("mm/s", "mm/s2") and "width" in base_cmd:
            motor = _motor_from_cmd(base_cmd)
            mpr = _get_mm_per_rev(cfg_data, motor)
            if mpr > 0:
                converted = raw * mpr / 60.0
                return f"{converted:.1f}"
            return "N/A"
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
        if _controller_rebooted[0]:
            _wait_and_restore(ser)
        cfg_data = _read_current_values(ser, node_idx)
        _clear_screen()
        node_name = NODES[node_idx][0]
        _print_header(f"{node_name} Parameters", f"{level_name} / {node_name}")

        if node_idx == 1:
            head_pos = _get_head_pos_mm(ser)
            if head_pos is not None:
                print(f"  Head Position: {head_pos:.2f} mm\n")
            else:
                print(f"  Head Position: ? mm\n")

        for i, (name, cmd, step, unit) in enumerate(params):
            val = _get_current_value(cfg_data, name, cmd, unit)
            print(f"  {i+1:2d}. {name:<20s} = {val} {unit}")

        print(f"\n  [1-{len(params)}] Select parameter")
        if node_idx == 1:
            if _board_clamped[0]:
                print("  [c] Release board")
            else:
                print("  [c] Clamp board")
            print("  [h] Home head")
        print("  [d] Restore defaults")
        print("  [s] Save to flash")
        print("  [q] Back\n")

        key = _get_key(ser)
        if key in ('q', 'Q', '\x1b'):
            if node_idx == 1:
                _unclamp_board(ser)
            return
        if key in ('d', 'D'):
            send_and_capture(ser, "cfg defaults")
            print("  >> Defaults restored")
            time.sleep(1)
            continue
        if key in ('s', 'S'):
            send_and_capture(ser, "cfg save")
            print("  >> Saved to flash!")
            time.sleep(1)
            continue

        if node_idx == 1 and key in ('c', 'C'):
            if _board_clamped[0]:
                _unclamp_board(ser)
                print("  >> Board released")
            else:
                _clamp_board_sequence(ser, level_name)
            time.sleep(0.5)
            continue

        if node_idx == 1 and key in ('h', 'H'):
            send_and_capture(ser, "manual motor 3 on")
            send_and_capture(ser, "manual motor 3 home", timeout=15)
            print("  >> Head homed")
            time.sleep(1)
            continue

        if key and key.isdigit():
            idx = int(key) - 1
            if 0 <= idx < len(params):
                _adjust_param(ser, params[idx], node_idx, level_name)


_board_clamped = [False]


def _clamp_board_sequence(ser, level_name):
    """Interactive board clamping sequence before head position adjustment."""
    _clear_screen()
    _print_header("Board Clamping", f"{level_name} / Head")
    print("  Step 1: Raising stopper...")
    send_and_capture(ser, "manual cylinder stopper a", timeout=5)
    print("  >> Stopper UP\n")
    print("  Step 2: Place the board against the stopper")
    print("          Press any key when ready (q to cancel)...")
    key = _get_key(ser)
    if key in ('q', 'Q', '\x1b'):
        return False

    _clear_screen()
    _print_header("Board Clamping", f"{level_name} / Head")
    print("  Step 3: Raising RFID...")
    send_and_capture(ser, "manual cylinder rfid a", timeout=5)
    print("  >> RFID UP\n")
    print("  Step 4: Clamping board (locker)...")
    send_and_capture(ser, "manual cylinder locker b", timeout=5)
    print("  >> Board CLAMPED\n")
    print("  Board is secured. Press any key to continue...")
    _get_key(ser)
    _board_clamped[0] = True
    return True


def _unclamp_board(ser):
    """Release board after head position adjustment."""
    if _board_clamped[0]:
        send_and_capture(ser, "manual cylinder locker a", timeout=5)
        send_and_capture(ser, "manual cylinder rfid b", timeout=5)
        send_and_capture(ser, "manual cylinder stopper b", timeout=5)
        _board_clamped[0] = False


def _adjust_head_position(ser, param_def, node_idx, level_name, go_on_enter=True):
    """Adjust head position param by physically moving the head."""
    name, cmd_template, step, unit = param_def
    node_name = NODES[node_idx][0]

    cfg_data = _read_current_values(ser, node_idx)
    current = _get_current_value(cfg_data, name, cmd_template, unit)
    try:
        target_pos = float(current)
    except (ValueError, TypeError):
        target_pos = 0.0

    send_and_capture(ser, "manual motor 3 on")

    actual_pos = _get_head_pos_mm(ser)
    if actual_pos is None:
        actual_pos = 0.0

    if go_on_enter:
        delta = target_pos - actual_pos
        if abs(delta) > 0.01:
            send_and_capture(ser, f"manual motor 3 go_mm {delta} 100", timeout=10)
    else:
        target_pos = actual_pos

    while True:
        if _controller_rebooted[0]:
            _wait_and_restore(ser)
            return

        _clear_screen()
        _print_header(f"Adjust: {name}", f"{level_name} / {node_name} / {name}")
        print(f"  Position:  {target_pos:.2f} mm")
        print(f"  Step size: {step} mm")
        print()
        print("  [Up]           Move UP by step (+)")
        print("  [Down]         Move DOWN by step (-)")
        print("  [<] / [Left]   Decrease step size")
        print("  [>] / [Right]  Increase step size")
        print("  [v]            Enter position directly")
        print("  [h]            Home head (reset to 0)")
        print("  [q] / [Esc]    Save & Back\n")

        key = _get_key(ser)
        if key is None and _controller_rebooted[0]:
            continue

        if key in ('q', 'Q', '\x1b'):
            cmd = cmd_template.format(v=target_pos)
            send_and_capture(ser, cmd)
            return

        if key == 'up':
            delta = step
            target_pos += delta
            send_and_capture(ser, f"manual motor 3 go_mm {delta} 100", timeout=5)
            continue
        if key == 'down':
            delta = -step
            target_pos += delta
            send_and_capture(ser, f"manual motor 3 go_mm {delta} 100", timeout=5)
            continue
        if key == '<':
            step = max(0.1, step - 0.1)
            step = round(step, 1)
            continue
        if key == '>':
            step = round(step + 0.1, 1)
            continue
        if key in ('h', 'H'):
            send_and_capture(ser, "manual motor 3 home", timeout=15)
            target_pos = 0.0
            continue
        if key == 'v' or key == 'V':
            print(f"  Enter position (mm): ", end="", flush=True)
            val_str = ""
            while True:
                ch = _get_key(ser)
                if ch is None:
                    break
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
                    new_pos = float(val_str)
                    delta = new_pos - target_pos
                    send_and_capture(ser, f"manual motor 3 go_mm {delta} 100", timeout=5)
                    target_pos = new_pos
                except ValueError:
                    print("  Invalid value!")
                    time.sleep(1)


def _get_head_pos_mm(ser):
    """Read current head position in mm via a zero-distance go_mm."""
    lines = send_and_capture(ser, "manual motor 3 go_mm 0 1", timeout=2.0)
    for line in lines:
        m = re.search(r'([-\d.]+)\s*mm\)', line)
        if m:
            return float(m.group(1))
    return None


def _adjust_param(ser, param_def, node_idx, level_name):
    """Adjust a single parameter: direct value or < / > stepping."""
    name, cmd_template, step, unit = param_def
    base_cmd = cmd_template.split("{v}")[0].strip()
    motor = _motor_from_cmd(base_cmd)

    # Head position params — special mode with physical movement
    is_head_pos = motor == "head" and name in ("Guides Clear", "Pins Touch", "Pins Contact")
    is_head_limit = motor == "head" and name in ("Soft Limit Min", "Soft Limit Max")
    if is_head_pos or is_head_limit:
        return _adjust_head_position(ser, param_def, node_idx, level_name, go_on_enter=is_head_pos)

    node_name = NODES[node_idx][0]
    needs_rpm_conv = unit in ("mm/s", "mm/s2") and motor == "width"
    is_speed = unit in ("mm/s", "rpm")
    is_head_mm = motor == "head" and unit == "mm"
    motor_idx = {"conveyor": "1", "width": "2", "head": "3"}.get(motor, "1")
    motor_running = False

    def _apply_speed(mm_s, cfg_data, restart=False):
        """Send run command to motor at given mm/s speed."""
        try:
            if needs_rpm_conv:
                m = _get_mm_per_rev(cfg_data, motor)
                rpm = mm_s * 60.0 / m if m > 0 else mm_s
            elif motor in ("conveyor", "head"):
                m = _get_mm_per_rev(cfg_data, motor)
                rpm = mm_s * 60.0 / m if m > 0 else mm_s
            else:
                rpm = mm_s
            if restart:
                send_and_capture(ser, f"manual motor {motor_idx} stop")
            send_and_capture(ser, f"manual motor {motor_idx} run {int(rpm)} fwd")
        except (ValueError, TypeError, ZeroDivisionError):
            pass

    while True:
        if _controller_rebooted[0]:
            motor_running = False
            _wait_and_restore(ser)

        cfg_data = _read_current_values(ser, node_idx)
        mpr = _get_mm_per_rev(cfg_data, motor) if needs_rpm_conv else 0
        current = _get_current_value(cfg_data, name, cmd_template, unit)

        head_pos = None
        if is_head_mm:
            head_pos = _get_head_pos_mm(ser)

        _clear_screen()
        _print_header(f"Adjust: {name}", f"{level_name} / {node_name} / {name}")
        print(f"  Current value: {current} {unit}")
        if head_pos is not None:
            print(f"  Head position: {head_pos:.2f} mm")
        print(f"  Step size:     {step} {unit}")
        if is_speed and motor_running:
            print(f"  Motor:         RUNNING")
        print()
        print("  [<] / [Left]   Decrease by step")
        print("  [>] / [Right]  Increase by step")
        print("  [Up] / [Down]  Increase / Decrease step size")
        print("  [v]            Enter value directly")
        if is_speed:
            print("  [r]            Run motor    [x] Stop motor")
        if is_head_mm:
            print("  [g]            Go to current value")
        print("  [q] / [Esc]    Back\n")

        key = _get_key(ser)
        if key is None and _controller_rebooted[0]:
            motor_running = False
            continue
        if key in ('q', 'Q', '\x1b'):
            if is_speed and motor_running:
                send_and_capture(ser, f"manual motor {motor_idx} stop")
                motor_running = False
            return

        if is_speed and key in ('r', 'R'):
            try:
                _apply_speed(float(current), cfg_data)
                motor_running = True
            except (ValueError, TypeError):
                pass
            continue
        if is_speed and key in ('x', 'X'):
            send_and_capture(ser, f"manual motor {motor_idx} stop")
            motor_running = False
            continue

        if is_head_mm and key in ('g', 'G'):
            try:
                target = float(current)
                cur = head_pos if head_pos is not None else 0
                delta = target - cur
                send_and_capture(ser, f"manual motor 3 go_mm {delta} 100", timeout=10)
            except (ValueError, TypeError):
                pass
            motor_running = False
            continue

        if key == 'up':
            step = step * 2 if isinstance(step, float) else step + 1
            step = round(step, 2)
            continue
        if key == 'down':
            if isinstance(step, float):
                step = max(0.1, round(step / 2, 2))
            else:
                step = max(1, step - 1)
            continue

        new_mm_s = None

        if key == '<':
            try:
                new_val = float(current) - step
                if needs_rpm_conv and mpr > 0:
                    hw_val = int(round(new_val * 60.0 / mpr))
                elif step == int(step) and '.' not in str(current):
                    hw_val = int(new_val)
                else:
                    hw_val = new_val
                cmd = cmd_template.format(v=hw_val)
                send_and_capture(ser, cmd)
                new_mm_s = new_val
            except (ValueError, TypeError):
                pass

        elif key == '>':
            try:
                new_val = float(current) + step
                if needs_rpm_conv and mpr > 0:
                    hw_val = int(round(new_val * 60.0 / mpr))
                elif step == int(step) and '.' not in str(current):
                    hw_val = int(new_val)
                else:
                    hw_val = new_val
                cmd = cmd_template.format(v=hw_val)
                send_and_capture(ser, cmd)
                new_mm_s = new_val
            except (ValueError, TypeError):
                pass

        elif key == 'v' or key == 'V':
            print(f"  Enter new value ({unit}): ", end="", flush=True)
            val_str = ""
            while True:
                ch = _get_key(ser)
                if ch is None:
                    break
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
                    if needs_rpm_conv and mpr > 0:
                        hw_val = int(round(val * 60.0 / mpr))
                    elif step == int(step) and '.' not in val_str:
                        hw_val = int(val)
                    else:
                        hw_val = val
                    cmd = cmd_template.format(v=hw_val)
                    send_and_capture(ser, cmd)
                    new_mm_s = val
                except ValueError:
                    print("  Invalid value!")
                    time.sleep(1)

        if is_speed and motor_running and new_mm_s is not None:
            _apply_speed(new_mm_s, cfg_data, restart=True)


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

        key = _get_key(ser)
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

        key = _get_key(ser)
        if key in ('q', 'Q', '\x1b'):
            return
        if key == 'f':
            print("  Cylinder (1=stopper, 2=rfid, 3=locker): ", end="", flush=True)
            k = _get_key(ser)
            if k in ('1', '2', '3'):
                cyl = CYLINDER_NAMES[int(k)-1]
                print(f"{cyl}")
                print("  Position (a/b): ", end="", flush=True)
                p = _get_key(ser)
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

        key = _get_key(ser)
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


_unlock_cmd = [None]


def _wait_and_restore(ser):
    """Wait for controller to boot and re-send unlock command."""
    print("\n  >> Controller rebooted — waiting for ready...")
    deadline = time.time() + 15
    while not _controller_ready[0] and time.time() < deadline:
        _check_serial_for_boot(ser)
        time.sleep(0.05)
    _controller_rebooted[0] = False
    time.sleep(0.5)
    ser.reset_input_buffer()
    if _unlock_cmd[0]:
        send_and_capture(ser, _unlock_cmd[0])
    print("  >> Restored session")
    time.sleep(0.5)


def _setup_menu_inner(ser):
    """Internal setup menu logic."""
    # Level selection
    _clear_screen()
    _print_header("Setup Mode")
    print("  Access Level:")
    print("  1. Engineering (proc_eng)")
    print("  2. Factory")
    print("  q. Cancel\n")

    key = _get_key(ser)
    if key in ('q', 'Q', '\x1b'):
        return

    if key == '1':
        _unlock_cmd[0] = "cfg unlock proc_eng"
        send_and_capture(ser, _unlock_cmd[0])
        level_name = "Engineering"
    elif key == '2':
        _unlock_cmd[0] = "cfg unlock factory"
        send_and_capture(ser, _unlock_cmd[0])
        level_name = "Factory"
    else:
        return

    # Node selection loop
    while True:
        if _controller_rebooted[0]:
            _wait_and_restore(ser)
        _clear_screen()
        _print_header("Select Node", level_name)
        for i, (name, _) in enumerate(NODES):
            print(f"  {i+1}. {name}")
        print("\n  [s] Save to flash")
        print("  [q] Exit setup\n")

        key = _get_key(ser)
        if key in ('q', 'Q', '\x1b'):
            send_and_capture(ser, "cfg lock")
            _unlock_cmd[0] = None
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

    ser_holder = [ser]
    running = [True]
    tcp_clients = []
    tcp_lock = threading.Lock()

    reader = threading.Thread(target=reader_thread,
                              args=(ser_holder, running, tcp_clients, tcp_lock), daemon=True)
    reader.start()

    tcp_srv = threading.Thread(target=tcp_server_thread,
                               args=(ser_holder, running, tcp_port, tcp_clients, tcp_lock), daemon=True)
    tcp_srv.start()

    try:
        if config_file:
            send_config(ser, config_file, delay_ms)

        console_loop(ser_holder, running)
    finally:
        running[0] = False
        ser.close()


if __name__ == "__main__":
    main()
