# Jetson-STM32 packet protocol v1

Status: **Task 2 specification**

This document defines the binary link between the Jetson and the STM32. It
does not define motor mixing or sensor drivers. All multi-byte integers are
little-endian. In-memory C structures must be serialized field by field; they
must never be copied directly onto the wire.

## Common frame

| Byte offset | Size | Field | Value / meaning |
|---:|---:|---|---|
| 0 | 1 | SOF 1 | `0xAA` |
| 1 | 1 | SOF 2 | `0x55` |
| 2 | 1 | Version | `0x01` |
| 3 | 1 | Message type | `0x01` command, `0x02` telemetry |
| 4 | 1 | Payload length | Number of payload bytes, maximum 64 |
| 5 | 1 | Sequence | Increments modulo 256 |
| 6 | N | Payload | Defined by message type |
| 6 + N | 1 | CRC low | Low byte of CRC-16 |
| 7 + N | 1 | CRC high | High byte of CRC-16 |

Total frame length is `N + 8` bytes.

CRC parameters:

- Algorithm: CRC-16/CCITT-FALSE
- Polynomial: `0x1021`
- Initial value: `0xFFFF`
- Reflected input/output: no
- Final XOR: `0x0000`
- Covered bytes: Version through the final payload byte (frame offsets 2 to
  `5 + N`). The two SOF bytes and the CRC bytes are not included.

The receiver accepts a frame only when the version, message type, payload
length, and CRC are all valid. A rejected frame must not update control
setpoints or the link-loss timestamp.

## Command packet: Jetson to STM32

Message type: `0x01`

Payload length: 14 bytes

| Payload offset | Size | Type | Field | Unit / range |
|---:|---:|---|---|---|
| 0 | 2 | `uint16_t` | `control_flags` | Bit field defined below |
| 2 | 2 | `int16_t` | `surge_permille` | Forward/back, -1000 to +1000 |
| 4 | 2 | `int16_t` | `sway_permille` | Left/right, -1000 to +1000 |
| 6 | 2 | `int16_t` | `heave_permille` | Down/up, -1000 to +1000 |
| 8 | 2 | `int16_t` | `yaw_permille` | Turn, -1000 to +1000 |
| 10 | 4 | `int32_t` | `depth_setpoint_mm` | Desired depth in millimetres |

Command flag bits:

| Bit | Name | Meaning when set |
|---:|---|---|
| 0 | `ARM_REQUEST` | Request armed operation |
| 1 | `DEPTH_HOLD_ENABLE` | Use `depth_setpoint_mm`; ignore manual heave |
| 2 | `EMERGENCY_STOP` | Immediately request the safe state |
| 3-15 | Reserved | Sender writes zero; receiver ignores |

When depth hold is disabled, `depth_setpoint_mm` is ignored. When depth hold is
enabled, `heave_permille` is ignored by the controller.

Example command frame (`ARM_REQUEST`, surge `+250`, sway `-100`, heave `0`,
yaw `+50`, depth setpoint `1500 mm`, sequence `0x2A`):

```text
AA 55 01 01 0E 2A 01 00 FA 00 9C FF 00 00 32 00 DC 05 00 00 10 78
```

The final two bytes are CRC `0x7810`, sent low byte first.

## Telemetry packet: STM32 to Jetson

Message type: `0x02`

Payload length: 18 bytes

| Payload offset | Size | Type | Field | Unit / meaning |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `uptime_ms` | STM32 uptime in milliseconds |
| 4 | 4 | `int32_t` | `depth_mm` | Measured depth in millimetres |
| 8 | 2 | `int16_t` | `roll_cdeg` | Roll in centidegrees |
| 10 | 2 | `int16_t` | `pitch_cdeg` | Pitch in centidegrees |
| 12 | 2 | `int16_t` | `yaw_cdeg` | Yaw in centidegrees |
| 14 | 2 | `uint16_t` | `status_flags` | Bit field defined below |
| 16 | 1 | `uint8_t` | `last_command_sequence` | Last accepted command sequence |
| 17 | 1 | `uint8_t` | Reserved | Sender writes zero |

Telemetry status bits:

| Bit | Name | Meaning when set |
|---:|---|---|
| 0 | `ARMED` | Control output is armed |
| 1 | `COMMAND_LINK_OK` | Valid commands are arriving before timeout |
| 2 | `FAILSAFE_ACTIVE` | Link-loss or emergency-stop safe state active |
| 3 | `DEPTH_VALID` | Depth value is valid |
| 4 | `ATTITUDE_VALID` | Roll and pitch are valid |
| 5 | `SENSOR_FAULT` | At least one required sensor has failed |
| 6 | `CONTROL_FAULT` | Controller reported an internal fault |
| 7 | `RX_CRC_ERROR_SEEN` | At least one bad command CRC was observed |
| 8 | `IMU_DETECTED` | MPU6050 acknowledged at I2C address `0x68` or `0x69` |
| 9 | `DEPTH_SENSOR_DETECTED` | MS5837 acknowledged at I2C address `0x76` |
| 10 | `YAW_VALID` | Yaw comes from a valid heading source |
| 11 | `MAGNETOMETER_DETECTED` | HW-290 magnetometer acknowledged at `0x0D` (QMC5883L) or `0x1E` (HMC5883L) |
| 12-15 | Reserved | Sender writes zero; receiver ignores |

Until sensors are connected, their fields are zero and their corresponding
valid flags remain clear.

The MPU6050 provides accelerometer and gyroscope data but no absolute heading
reference by itself. On the HW-290 module, the firmware exposes the companion
QMC5883L/HMC5883L through MPU6050 I2C bypass and uses it for tilt-compensated
yaw. `YAW_VALID` remains clear if that magnetometer is absent or stale. This
first yaw value is live but uncalibrated; installed-system hard-iron and
soft-iron calibration is still required. MS5837 depth is relative to the
average pressure of the first 20 valid samples after startup.

Example telemetry frame (uptime `123456 ms`, depth `1450 mm`, roll `-1.23 deg`,
pitch `2.10 deg`, yaw `35.99 deg`, status `0x001A`, last command `0x2A`,
telemetry sequence `0x9C`):

```text
AA 55 01 02 12 9C 40 E2 01 00 AA 05 00 00 85 FF D2 00 0F 0E 1A 00 2A 00 36 E1
```

The final two bytes are CRC `0xE136`, sent low byte first.

## Shared control interface

The UART receive interrupt only moves bytes into the existing FreeRTOS queue.
It never parses packets and never locks a mutex.

The communications task:

1. Removes bytes from the queue.
2. finds SOF and rebuilds one complete frame.
3. validates version, length, type, and CRC.
4. converts the command payload into `ControlCommand_t`.
5. calls `CommInterface_SetCommand()` and records the receive timestamp.

The control task calls `CommInterface_GetCommand()` to copy a consistent
snapshot. Sensor/control code calls `CommInterface_SetTelemetry()`, and the
communications task calls `CommInterface_GetTelemetry()` before building the
outgoing telemetry packet. One RTOS mutex protects both shared snapshots.

## Safety rules

- Invalid or incomplete frames never change the active command.
- Only a fully valid command refreshes the future 500 ms link-loss watchdog.
- `EMERGENCY_STOP` has priority over every motion field.
- The STM32 refreshes the link watchdog only after a fully valid command.
- If commands stop for more than 500 ms, or `EMERGENCY_STOP` is received, the
  active shared command is replaced atomically with neutral setpoints and arm
  is cleared. `COMMAND_LINK_OK` and `FAILSAFE_ACTIVE` report the result.
- When motor output support is enabled, entering failsafe also writes neutral
  PWM immediately. Sensor-only builds keep every ESC output physically locked.
