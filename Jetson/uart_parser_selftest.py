#!/usr/bin/env python3
"""Ask the STM32 to run the in-memory binary packet parser self-test."""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is missing. Install it with: sudo apt install python3-serial")
    sys.exit(2)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the STM32 command packet parser self-test."
    )
    parser.add_argument(
        "port",
        nargs="?",
        default="/dev/ttyUSB0",
        help="serial device (default: /dev/ttyUSB0)",
    )
    args = parser.parse_args()

    try:
        with serial.Serial(
            port=args.port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2.0,
            write_timeout=1.0,
        ) as uart:
            time.sleep(1.0)
            uart.reset_input_buffer()

            print(f"Connected to {args.port} at 115200 8N1")
            uart.write(b"SELFTEST\n")
            uart.flush()
            print("Jetson -> STM32: SELFTEST")

            response = uart.readline().decode("ascii", errors="replace").strip()
            if response == "SELFTEST PASS":
                print("STM32 -> Jetson: SELFTEST PASS")
                print("PASS: all five packet parser tests passed")
                return 0

            if response.startswith("SELFTEST FAIL"):
                print(f"STM32 -> Jetson: {response}")
                print("FAIL: the number identifies the failed parser test")
                return 1

            if response:
                print(f"FAIL: unexpected response {response!r}")
            else:
                print("FAIL: no reply from the STM32")
            return 1
    except serial.SerialException as error:
        print(f"Serial error: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
