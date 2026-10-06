#!/usr/bin/env python3
"""End-to-end Jetson to STM32 binary command packet test."""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is missing. Install it with: sudo apt install python3-serial")
    sys.exit(2)


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def command_frame(
    sequence: int,
    *,
    flags: int = 0x0001,
    surge: int = 250,
    sway: int = -100,
    heave: int = 0,
    yaw: int = 50,
    depth_mm: int = 1500,
) -> bytes:
    payload = struct.pack(
        "<Hhhhhi", flags, surge, sway, heave, yaw, depth_mm
    )
    header = struct.pack("<BBBB", 0x01, 0x01, len(payload), sequence)
    crc = crc16_ccitt_false(header + payload)
    return b"\xAA\x55" + header + payload + struct.pack("<H", crc)


def read_expected(uart: serial.Serial, expected: str) -> bool:
    """Find an ASCII acknowledgement in a stream containing binary telemetry."""
    target = expected.encode("ascii")
    received = bytearray()
    deadline = time.monotonic() + 2.0

    while time.monotonic() < deadline:
        chunk = uart.read(max(1, uart.in_waiting))
        if chunk:
            received.extend(chunk)
            if target in received:
                print(f"STM32 -> Jetson: {expected}")
                return True

            if len(received) > 4096:
                del received[:-2048]

    print("STM32 -> Jetson: <expected response not found in mixed UART stream>")
    print(f"FAIL: expected {expected!r}")
    return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Test real binary command packets over USART6."
    )
    parser.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, 115200, timeout=2.0, write_timeout=1.0) as uart:
            time.sleep(1.0)
            uart.reset_input_buffer()
            print(f"Connected to {args.port} at 115200 8N1")

            print("Test 1: noise followed by a valid command")
            uart.write(b"\x00\x12\x77" + command_frame(0x31))
            uart.flush()
            if not read_expected(uart, "CMD OK SEQ=31"):
                return 1

            print("Test 2: command with a corrupted CRC")
            bad_crc = bytearray(command_frame(0x32))
            bad_crc[-1] ^= 0x01
            uart.write(bad_crc)
            uart.flush()
            if not read_expected(uart, "CMD ERROR CRC"):
                return 1

            print("Test 3: command header with an invalid payload length")
            uart.write(b"\xAA\x55\x01\x01\x0F\x33")
            uart.flush()
            if not read_expected(uart, "CMD ERROR LENGTH"):
                return 1

            print("Test 4: partial command, timeout, then a valid command")
            partial = command_frame(0x34)
            uart.write(partial[:10])
            uart.flush()
            time.sleep(0.2)
            uart.write(command_frame(0x35))
            uart.flush()
            if not read_expected(uart, "CMD OK SEQ=35"):
                return 1

            print("Test 5: valid CRC but an out-of-range motion value")
            uart.write(command_frame(0x36, surge=1500))
            uart.flush()
            if not read_expected(uart, "CMD ERROR RANGE"):
                return 1

            print("PASS: real UART command receive path works end to end")
            return 0
    except serial.SerialException as error:
        print(f"Serial error: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
