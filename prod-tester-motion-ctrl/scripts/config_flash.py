"""
Config blob flash utility for production-tester motion controller.

Usage:
  1. Capture config hex from device:
       cfg dump bin   (in device terminal — copy the hex lines between #CFG_BIN_V1 and #END)

  2. Convert to .bin file:
       python scripts/config_flash.py create <hex_dump_file> [output.bin]

  3. Flash to device:
       python scripts/config_flash.py flash [config.bin]
       (uses STM32_Programmer_CLI at partition address 0x08080000)

  4. All-in-one (create + flash):
       python scripts/config_flash.py create-flash <hex_dump_file>
"""

import sys
import os
import subprocess
import struct

CONFIG_FLASH_ADDR = 0x08080000
STM32_PROG = r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"


def parse_hex_dump(filepath):
    """Parse hex dump file (lines of hex between #CFG_BIN_V1 and #END)."""
    data = bytearray()
    capturing = False

    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if line.startswith("#CFG_BIN_V1"):
                capturing = True
                continue
            if line == "#END":
                break
            if not capturing:
                continue
            if line.startswith("#"):
                continue
            try:
                data.extend(bytes.fromhex(line))
            except ValueError:
                pass

    if len(data) == 0:
        print("ERROR: No hex data found in file.")
        sys.exit(1)

    magic = struct.unpack_from("<I", data, 0)[0]
    if magic != 0x43464731:
        print(f"WARNING: Bad magic 0x{magic:08X} (expected 0x43464731 'CFG1')")

    print(f"Parsed {len(data)} bytes from hex dump.")
    return bytes(data)


def cmd_create(args):
    if len(args) < 1:
        print("usage: config_flash.py create <hex_dump_file> [output.bin]")
        sys.exit(1)

    hex_file = args[0]
    out_file = args[1] if len(args) > 1 else "config.bin"

    data = parse_hex_dump(hex_file)

    with open(out_file, "wb") as f:
        f.write(data)

    print(f"Written {len(data)} bytes to {out_file}")
    return out_file


def cmd_flash(args):
    bin_file = args[0] if len(args) > 0 else "config.bin"

    if not os.path.exists(bin_file):
        print(f"ERROR: {bin_file} not found.")
        sys.exit(1)

    if not os.path.exists(STM32_PROG):
        print(f"ERROR: STM32_Programmer_CLI not found at: {STM32_PROG}")
        print("Set STM32_PROG environment variable or install STM32CubeProgrammer.")
        sys.exit(1)

    cmd = [
        STM32_PROG,
        "-c", "port=SWD",
        "-w", bin_file, f"0x{CONFIG_FLASH_ADDR:08X}",
        "-v",
    ]
    print(f"Flashing {bin_file} at 0x{CONFIG_FLASH_ADDR:08X}...")
    print(f"Command: {' '.join(cmd)}")
    result = subprocess.run(cmd)
    sys.exit(result.returncode)


def cmd_create_flash(args):
    bin_file = cmd_create(args)
    cmd_flash([bin_file])


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    command = sys.argv[1]
    args = sys.argv[2:]

    if command == "create":
        cmd_create(args)
    elif command == "flash":
        cmd_flash(args)
    elif command == "create-flash":
        cmd_create_flash(args)
    else:
        print(f"Unknown command: {command}")
        print(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    main()
