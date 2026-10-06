#!/usr/bin/env python3
"""Prove the STM32 500 ms command link-loss failsafe over the real UART."""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is missing. Install it with: sudo apt install python3-serial")
    sys.exit(2)


SOF = b"\xAA\x55"
STATUS_ARMED = 1 << 0
STATUS_COMMAND_LINK_OK = 1 << 1
STATUS_FAILSAFE_ACTIVE = 1 << 2


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


def command_frame(sequence: int) -> bytes:
    payload = struct.pack("<Hhhhhi", 0x0001, 250, -100, 0, 50, 1500)
    header = struct.pack("<BBBB", 0x01, 0x01, len(payload), sequence)
    crc = crc16_ccitt_false(header + payload)
    return SOF + header + payload + struct.pack("<H", crc)


def extract_telemetry(buffer: bytearray):
    while True:
        start = buffer.find(SOF)
        if start < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            return None
        if start:
            del buffer[:start]
        if len(buffer) < 6:
            return None

        version, message_type, payload_length, sequence = buffer[2:6]
        frame_size = 8 + payload_length
        if len(buffer) < frame_size:
            return None

        frame = bytes(buffer[:frame_size])
        del buffer[:frame_size]
        if version != 1 or message_type != 2 or payload_length != 18:
            continue

        received_crc = struct.unpack("<H", frame[-2:])[0]
        if received_crc != crc16_ccitt_false(frame[2:-2]):
            continue

        values = struct.unpack("<IihhhHBB", frame[6:-2])
        return sequence, values


def read_available(uart: serial.Serial, buffer: bytearray) -> list[tuple]:
    waiting = uart.in_waiting
    if waiting:
        buffer.extend(uart.read(waiting))
    packets = []
    while True:
        packet = extract_telemetry(buffer)
        if packet is None:
            return packets
        packets.append(packet)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, 115200, timeout=0.05,
                           write_timeout=1.0) as uart:
            time.sleep(0.5)
            uart.reset_input_buffer()
            stream = bytearray()
            sequence = 0x40
            saw_healthy_link = False

            print(f"Connected to {args.port} at 115200 8N1")
            print("Phase 1: sending valid commands at 10 Hz for 2 seconds")
            phase_start = time.monotonic()
            next_send = phase_start
            while time.monotonic() - phase_start < 2.0:
                now = time.monotonic()
                if now >= next_send:
                    uart.write(command_frame(sequence))
                    uart.flush()
                    sequence = (sequence + 1) & 0xFF
                    next_send += 0.1

                for _, values in read_available(uart, stream):
                    status = values[5]
                    if ((status & STATUS_COMMAND_LINK_OK) and
                            not (status & STATUS_FAILSAFE_ACTIVE)):
                        saw_healthy_link = True
                time.sleep(0.005)

            if not saw_healthy_link:
                print("FAIL: STM32 never reported a healthy command link")
                return 1

            print("PASS: COMMAND_LINK_OK=1 and FAILSAFE_ACTIVE=0 while commands arrive")
            print("Phase 2: stopping commands and waiting for the STM32 timeout")
            stopped_at = time.monotonic()
            deadline = stopped_at + 2.0
            while time.monotonic() < deadline:
                for _, values in read_available(uart, stream):
                    status = values[5]
                    last_command = values[6]
                    if (not (status & STATUS_COMMAND_LINK_OK) and
                            (status & STATUS_FAILSAFE_ACTIVE)):
                        elapsed = time.monotonic() - stopped_at
                        armed = bool(status & STATUS_ARMED)
                        print(
                            f"Failsafe reported after {elapsed:.3f} s: "
                            f"status=0x{status:04X}, armed={int(armed)}, "
                            f"last_cmd={last_command}"
                        )
                        if armed:
                            print("FAIL: ARMED remained set during failsafe")
                            return 1
                        if not 0.35 <= elapsed <= 0.85:
                            print("FAIL: failsafe transition was outside the expected window")
                            return 1
                        print("PASS: link loss cleared arm and activated neutral failsafe")
                        return 0
                time.sleep(0.005)

            print("FAIL: failsafe did not activate within 2 seconds")
            return 1
    except serial.SerialException as error:
        print(f"Serial error: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
