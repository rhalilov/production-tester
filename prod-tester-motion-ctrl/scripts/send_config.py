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

import serial

DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "default_config.txt")
DEFAULT_BAUD = 115200
DEFAULT_DELAY_MS = 50
DEFAULT_TCP_PORT = 5555


def reader_thread(ser, running, tcp_clients, tcp_lock):
    """Background thread that prints incoming serial data and forwards to TCP clients."""
    while running[0]:
        try:
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
    print("--- Console active (type 'exit' or 'quit' to close) ---")
    while True:
        try:
            line = input()
            if line.strip().lower() in ("exit", "quit"):
                print("\n--- Disconnected ---")
                break
            ser.write((line + "\r\n").encode())
        except KeyboardInterrupt:
            print()
            continue
        except EOFError:
            print("\n--- Disconnected ---")
            break


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
