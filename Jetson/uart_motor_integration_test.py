#!/usr/bin/env python3
"""Exercise all motion axes and both STM32 stop mechanisms over real UART."""

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
ARM_REQUEST = 1 << 0
EMERGENCY_STOP = 1 << 2
STATUS_ARMED = 1 << 0
STATUS_COMMAND_LINK_OK = 1 << 1
STATUS_FAILSAFE_ACTIVE = 1 << 2
STATUS_SENSOR_MASK = 0x0F18
COMMAND_PERIOD_S = 0.1
MOTION_LEVEL = 170


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) if crc & 0x8000 else
                   (crc << 1)) & 0xFFFF
    return crc


def command_frame(sequence: int, *, flags: int = ARM_REQUEST,
                  surge: int = 0, sway: int = 0, heave: int = 0,
                  yaw: int = 0) -> bytes:
    payload = struct.pack("<Hhhhhi", flags, surge, sway, heave, yaw, 0)
    header = struct.pack("<BBBB", 1, 1, len(payload), sequence)
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
        if struct.unpack("<H", frame[-2:])[0] != crc16_ccitt_false(frame[2:-2]):
            continue

        values = struct.unpack("<IihhhHBB", frame[6:-2])
        return {
            "sequence": sequence,
            "uptime_ms": values[0],
            "depth_mm": values[1],
            "roll_cdeg": values[2],
            "pitch_cdeg": values[3],
            "yaw_cdeg": values[4],
            "status": values[5],
            "last_command": values[6],
        }


def read_telemetry(uart, stream: bytearray):
    waiting = uart.in_waiting
    if waiting:
        stream.extend(uart.read(waiting))
    newest = None
    while True:
        packet = extract_telemetry(stream)
        if packet is None:
            return newest
        newest = packet


def send_phase(uart, stream: bytearray, sequence: int, name: str,
               duration: float, **command):
    print(f"  {name} ({duration:.1f} s)")
    deadline = time.monotonic() + duration
    next_send = time.monotonic()
    newest = None
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send:
            uart.write(command_frame(sequence, **command))
            uart.flush()
            sequence = (sequence + 1) & 0xFF
            next_send += COMMAND_PERIOD_S
        packet = read_telemetry(uart, stream)
        if packet is not None:
            newest = packet
        time.sleep(0.005)
    return sequence, newest


def require_armed(packet, phase: str):
    if packet is None:
        raise RuntimeError(f"no telemetry received during {phase}")
    status = packet["status"]
    if not (status & STATUS_ARMED and status & STATUS_COMMAND_LINK_OK):
        raise RuntimeError(
            f"STM32 was not armed with a healthy link during {phase}: "
            f"status=0x{status:04X}"
        )
    if status & STATUS_FAILSAFE_ACTIVE:
        raise RuntimeError(
            f"failsafe was unexpectedly active during {phase}: "
            f"status=0x{status:04X}"
        )
    if (status & STATUS_SENSOR_MASK) != STATUS_SENSOR_MASK:
        raise RuntimeError(
            f"live sensor flags were incomplete during {phase}: "
            f"status=0x{status:04X}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    args = parser.parse_args()

    print("INTEGRATED MOTOR TEST")
    print("- Secure the ROV and keep hands, cables, and tools away from all props.")
    print("- Keep the battery disconnect within immediate reach.")
    print("- The STM32 validation build limits PWM to 1500 +/- 60 us.")
    confirmation = input("Type RUN to enable motion: ").strip()
    if confirmation != "RUN":
        print("Cancelled: no motor command was sent")
        return 2

    uart = None
    sequence = 0x80
    stream = bytearray()
    try:
        uart = serial.Serial(args.port, 115200, timeout=0.05,
                             write_timeout=1.0)
        time.sleep(0.5)
        uart.reset_input_buffer()
        print(f"Connected to {args.port} at 115200 8N1")

        print("Phase 1/4: arm at neutral and verify live sensors")
        sequence, packet = send_phase(
            uart, stream, sequence, "ARMED NEUTRAL", 0.8
        )
        require_armed(packet, "armed neutral")
        print(
            "  PASS: armed, link healthy, depth/roll/pitch/yaw telemetry live"
        )

        print("Phase 2/4: exercise the complete mixer at low power")
        patterns = [
            ("HEAVE: four vertical thrusters", {"heave": MOTION_LEVEL}),
            ("SURGE: four horizontal thrusters", {"surge": MOTION_LEVEL}),
            ("SWAY: horizontal diagonal mix", {"sway": MOTION_LEVEL}),
            ("YAW: horizontal turning mix", {"yaw": MOTION_LEVEL}),
        ]
        for name, command in patterns:
            sequence, packet = send_phase(
                uart, stream, sequence, name, 0.6, **command
            )
            require_armed(packet, name)
            sequence, packet = send_phase(
                uart, stream, sequence, "neutral pause", 0.5
            )
            require_armed(packet, f"neutral after {name}")
        print("  PASS: all four mixer axes completed with healthy telemetry")

        print("Phase 3/4: prove immediate Emergency Stop")
        sequence, packet = send_phase(
            uart, stream, sequence, "brief heave", 0.3,
            heave=MOTION_LEVEL
        )
        require_armed(packet, "pre-emergency motion")
        sequence, packet = send_phase(
            uart, stream, sequence, "EMERGENCY STOP", 0.5,
            flags=EMERGENCY_STOP
        )
        if packet is None:
            raise RuntimeError("no telemetry after Emergency Stop")
        if ((packet["status"] & STATUS_ARMED) or
                not (packet["status"] & STATUS_FAILSAFE_ACTIVE)):
            raise RuntimeError(
                "Emergency Stop did not disarm into failsafe: "
                f"status=0x{packet['status']:04X}"
            )
        print("  PASS: Emergency Stop cleared ARMED and forced neutral")

        sequence, packet = send_phase(
            uart, stream, sequence, "re-arm at neutral", 0.7
        )
        require_armed(packet, "re-arm after Emergency Stop")

        print("Phase 4/4: prove the 500 ms UART link-loss failsafe physically")
        sequence, packet = send_phase(
            uart, stream, sequence, "heave before link loss", 0.3,
            heave=MOTION_LEVEL
        )
        require_armed(packet, "pre-link-loss motion")
        print("  UART commands stopped; thrusters must return to neutral automatically")
        stopped_at = time.monotonic()
        deadline = stopped_at + 1.5
        failsafe_packet = None
        while time.monotonic() < deadline:
            packet = read_telemetry(uart, stream)
            if packet is not None:
                status = packet["status"]
                if (status & STATUS_FAILSAFE_ACTIVE and
                        not status & STATUS_COMMAND_LINK_OK and
                        not status & STATUS_ARMED):
                    failsafe_packet = packet
                    break
            time.sleep(0.005)

        if failsafe_packet is None:
            raise RuntimeError("link-loss failsafe did not activate")
        elapsed = time.monotonic() - stopped_at
        if not 0.35 <= elapsed <= 0.85:
            raise RuntimeError(
                f"link-loss transition took {elapsed:.3f} s; expected about 0.5 s"
            )
        print(f"  PASS: link loss forced neutral after {elapsed:.3f} s")
        print("PASS: integrated commands, sensors, mixer, E-stop, and failsafe work")
        print("Confirm visually that each commanded thruster group moved and stopped.")
        return 0

    except (serial.SerialException, RuntimeError) as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        if uart is not None and uart.is_open:
            try:
                for _ in range(3):
                    uart.write(command_frame(sequence, flags=EMERGENCY_STOP))
                    uart.flush()
                    sequence = (sequence + 1) & 0xFF
                    time.sleep(COMMAND_PERIOD_S)
            except serial.SerialException:
                pass
            uart.close()
        print("Final state requested: EMERGENCY STOP / neutral")


if __name__ == "__main__":
    raise SystemExit(main())
