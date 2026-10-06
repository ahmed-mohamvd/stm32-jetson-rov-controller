#!/usr/bin/env python3
"""Minimal Jetson <-> STM32 USART6 communication test."""

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
        description="Send PING to the STM32 and wait for PONG."
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
            timeout=1.0,
            write_timeout=1.0,
        ) as uart:
            # Opening some USB-UART adapters can reset the target. Give it time
            # to boot, then discard any incomplete bytes left in the buffer.
            time.sleep(1.0)
            waiting = uart.read(uart.in_waiting or 1).decode("ascii", errors="replace")
            if waiting:
                print(f"STM32 startup: {waiting.strip()}")

            print(f"Connected to {args.port} at 115200 8N1")
            uart.write(b"PING\n")
            uart.flush()
            print("Jetson -> STM32: PING")

            response = uart.readline().decode("ascii", errors="replace").strip()
            if response == "PONG":
                print("STM32 -> Jetson: PONG")
                print("PASS: bidirectional UART communication works")
                return 0

            if response:
                print(f"FAIL: expected PONG, received {response!r}")
            else:
                print("FAIL: no reply from the STM32")
            return 1
    except serial.SerialException as error:
        print(f"Serial error: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
