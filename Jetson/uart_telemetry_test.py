#!/usr/bin/env python3
"""Receive, validate, and decode STM32 binary telemetry frames."""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is missing. Install it with: sudo apt install python3-serial")
    sys.exit(2)


SOF_1 = 0xAA
SOF_2 = 0x55
PROTOCOL_VERSION = 0x01
TELEMETRY_TYPE = 0x02
TELEMETRY_PAYLOAD_SIZE = 18


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


def read_exact(uart: serial.Serial, size: int) -> bytes | None:
    data = bytearray()
    while len(data) < size:
        chunk = uart.read(size - len(data))
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)


def find_start(uart: serial.Serial) -> bool:
    saw_first = False
    while True:
        value = uart.read(1)
        if not value:
            return False

        byte = value[0]
        if not saw_first:
            saw_first = byte == SOF_1
        elif byte == SOF_2:
            return True
        else:
            saw_first = byte == SOF_1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate STM32 telemetry packets and their 10 Hz rate."
    )
    parser.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    parser.add_argument("--count", type=int, default=20)
    args = parser.parse_args()

    if args.count < 2:
        parser.error("--count must be at least 2")

    try:
        with serial.Serial(args.port, 115200, timeout=2.0) as uart:
            time.sleep(0.5)
            uart.reset_input_buffer()
            print(f"Connected to {args.port} at 115200 8N1")
            print(f"Waiting for {args.count} valid telemetry packets...")

            receive_times: list[float] = []
            sequences: list[int] = []
            uptimes: list[int] = []
            crc_errors = 0
            header_errors = 0

            while len(sequences) < args.count:
                if not find_start(uart):
                    print("FAIL: timed out waiting for telemetry SOF")
                    return 1

                header = read_exact(uart, 4)
                if header is None:
                    print("FAIL: timed out while reading telemetry header")
                    return 1

                version, message_type, payload_length, sequence = header
                if (
                    version != PROTOCOL_VERSION
                    or message_type != TELEMETRY_TYPE
                    or payload_length != TELEMETRY_PAYLOAD_SIZE
                ):
                    header_errors += 1
                    continue

                tail = read_exact(uart, payload_length + 2)
                if tail is None:
                    print("FAIL: timed out while reading telemetry payload")
                    return 1

                payload = tail[:-2]
                received_crc = struct.unpack("<H", tail[-2:])[0]
                calculated_crc = crc16_ccitt_false(header + payload)
                if received_crc != calculated_crc:
                    crc_errors += 1
                    continue

                (
                    uptime_ms,
                    depth_mm,
                    roll_cdeg,
                    pitch_cdeg,
                    yaw_cdeg,
                    status_flags,
                    last_command_sequence,
                    reserved,
                ) = struct.unpack("<IihhhHBB", payload)

                if reserved != 0:
                    print(f"FAIL: reserved telemetry byte is {reserved}, expected 0")
                    return 1

                receive_times.append(time.monotonic())
                sequences.append(sequence)
                uptimes.append(uptime_ms)

                print(
                    f"seq={sequence:3d}  uptime={uptime_ms:8d} ms  "
                    f"depth={depth_mm:5d} mm  "
                    f"roll={roll_cdeg / 100:7.2f} deg  "
                    f"pitch={pitch_cdeg / 100:7.2f} deg  "
                    f"yaw={yaw_cdeg / 100:7.2f} deg  "
                    f"status=0x{status_flags:04X}  "
                    f"last_cmd={last_command_sequence}"
                )

            for previous, current in zip(sequences, sequences[1:]):
                if current != ((previous + 1) & 0xFF):
                    print(
                        f"FAIL: telemetry sequence jumped from {previous} to {current}"
                    )
                    return 1

            for previous, current in zip(uptimes, uptimes[1:]):
                if current <= previous:
                    print(
                        f"FAIL: uptime did not increase ({previous} -> {current})"
                    )
                    return 1

            elapsed = receive_times[-1] - receive_times[0]
            measured_rate = (len(receive_times) - 1) / elapsed
            print(
                f"Summary: valid={len(sequences)}, crc_errors={crc_errors}, "
                f"header_errors={header_errors}, rate={measured_rate:.2f} Hz"
            )

            if not 8.0 <= measured_rate <= 12.0:
                print("FAIL: telemetry rate is outside the accepted 8-12 Hz range")
                return 1

            print("PASS: telemetry receive, CRC, sequence, and rate are correct")
            return 0
    except serial.SerialException as error:
        print(f"Serial error: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
