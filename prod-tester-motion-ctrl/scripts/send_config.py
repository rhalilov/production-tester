"""
Serial console and configuration tool for the motion controller.

Usage:
  python scripts/send_config.py <COM_PORT> [config_file] [--delay=MS]
  python scripts/send_config.py <COM_PORT> --dump [output_file]

Modes:
  With config file:  Sends all commands from file, then enters console mode.
  Without config:    Opens interactive console directly.
  With --dump:       Reads config from board and saves to file.

Examples:
  python scripts/send_config.py COM19                              # Console only
  python scripts/send_config.py COM19 config/default_config.txt    # Send config, then console
  python scripts/send_config.py COM19 --dump                       # Dump to default_config.txt
  python scripts/send_config.py COM19 --dump my_config.txt         # Dump to custom file

Console:
  - Type commands and press Enter to send
  - Ctrl+C to exit
"""

import sys
import time
import threading
import os
import re

import serial

DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "default_config.txt")
DEFAULT_BAUD = 115200
DEFAULT_DELAY_MS = 50


def reader_thread(ser, running):
    """Background thread that prints incoming serial data."""
    while running[0]:
        try:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                text = data.decode(errors="replace")
                sys.stdout.write(text)
                sys.stdout.flush()
            else:
                time.sleep(0.02)
        except (OSError, serial.SerialException):
            break


def send_config(ser, config_file, delay_ms):
    """Send config file lines to the serial port."""
    with open(config_file, "r") as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith("#")]

    print(f"\n--- Sending {len(lines)} commands (delay={delay_ms}ms) ---")
    for i, line in enumerate(lines):
        ser.write((line + "\r\n").encode())
        time.sleep(delay_ms / 1000.0)
        time.sleep(0.05)
    print(f"--- All {len(lines)} commands sent ---\n")


def dump_config(ser, output_file):
    """Send 'cfg dump' to board, capture output, save as loadable config file."""
    ser.reset_input_buffer()
    ser.write(b"cfg dump\r\n")

    # Collect response lines
    lines = []
    deadline = time.time() + 5.0
    capturing = False

    while time.time() < deadline:
        if ser.in_waiting:
            raw = ser.readline().decode(errors="replace").strip()
            # Strip ANSI escape codes
            clean = re.sub(r'\x1b\[[0-9;]*m', '', raw)
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

    # Write as loadable config file
    with open(output_file, "w") as f:
        f.write("cfg unlock factory\n")
        for line in lines:
            f.write(f"cfg load {line}\n")
        f.write("cfg save\n")

    print(f"--- Dumped {len(lines)} parameters to {output_file} ---")
    return True


def console(ser):
    """Interactive console — read user input, send to serial."""
    print("--- Console active (Ctrl+C to exit) ---")
    try:
        while True:
            line = input()
            ser.write((line + "\r\n").encode())
    except (KeyboardInterrupt, EOFError):
        print("\n--- Disconnected ---")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    port = sys.argv[1]
    config_file = None
    delay_ms = DEFAULT_DELAY_MS
    dump_mode = False
    dump_output = DEFAULT_CONFIG

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
    reader = threading.Thread(target=reader_thread, args=(ser, running), daemon=True)
    reader.start()

    try:
        if config_file:
            send_config(ser, config_file, delay_ms)

        console(ser)
    finally:
        running[0] = False
        ser.close()


if __name__ == "__main__":
    main()
