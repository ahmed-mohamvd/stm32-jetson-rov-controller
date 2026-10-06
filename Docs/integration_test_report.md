# STM32F411–Jetson Communication Integration Test Report

**Test date:** 2026-10-06  
**STM32 target:** STM32F411CEU6 BlackPill  
**Host:** NVIDIA Jetson, Ubuntu 24.04, `/dev/ttyUSB0` through CH340  
**UART:** USART6 on PA11/PA12, 115200 baud, 8N1  
**Firmware mode:** integrated hardware validation; commanded motor outputs
enabled with a hard `1500 +/- 60 us` PWM limit

## 1. Scope

This report verifies the packet protocol, STM32 UART receive path, Jetson UART
integration, live telemetry from the installed sensors, motor mixer, PWM
output path, Emergency Stop, and the 500 ms command-link failsafe. The test
firmware runs communication, sensor, telemetry, control-safety, and motor
output tasks together under FreeRTOS.

All eight thrusters were first tested individually at `1500 +/- 50 us`. The
integrated build then enabled binary command control while applying a final
hardware-boundary clamp of `1500 +/- 60 us`. This prevents a valid but
incorrect full-scale command from producing full thrust during dry testing.

## 2. Build result

| Item | Result |
|---|---:|
| Build configuration | Debug |
| Compiler errors | 0 |
| Compiler/linker warnings | 1 |
| Warning | ELF LOAD segment has RWX permissions |
| Text | 75,120 bytes |
| Data | 108 bytes |
| BSS | 22,128 bytes |
| Total size report | 97,356 bytes (`0x17C4C`) |

The RWX warning is a linker-script permission warning and did not prevent
programming or execution.

## 3. Hardware and live sensor sources

| Measurement | Source | Result |
|---|---|---|
| Roll and pitch | MPU6050 accelerometer + gyroscope | Live and valid |
| Yaw | HW-290 QMC5883L/HMC5883L magnetometer path | Live and valid |
| Depth | MS5837 pressure sensor | Live, boot-relative depth |
| Transport | CH340 USB-to-TTL adapter | Stable at 115200 baud |

The magnetometer is accessed through MPU6050 auxiliary-I2C bypass. The
firmware automatically checks the QMC5883L address `0x0D` and the HMC5883L
address `0x1E`.

## 4. Test results

### 4.1 Packet parser self-test

Command:

```text
python3 ~/uart_parser_selftest.py /dev/ttyUSB0
```

Result:

```text
SELFTEST PASS
PASS: all five packet parser tests passed
```

Verified acceptance of a correct packet and rejection/recovery for corrupted
CRC, noise, partial input, and an invalid payload length.

### 4.2 Real UART command receive path

Command:

```text
python3 ~/uart_command_test.py /dev/ttyUSB0
```

Result: **PASS**.

| Test | Observed STM32 response |
|---|---|
| Noise followed by a valid command | `CMD OK SEQ=31` |
| Corrupted CRC | `CMD ERROR CRC` |
| Invalid payload length | `CMD ERROR LENGTH` |
| Partial frame, timeout, then valid frame | `CMD OK SEQ=35` |
| Motion value outside `-1000..1000` | `CMD ERROR RANGE` |

The Jetson test reader was made stream-aware so binary telemetry can coexist
with ASCII diagnostic acknowledgements on the same UART.

### 4.3 Live telemetry path

Command:

```text
python3 ~/uart_telemetry_test.py /dev/ttyUSB0
```

Observed result:

| Metric | Result |
|---|---:|
| Valid packets | 20 |
| CRC errors | 0 |
| Header errors | 0 |
| Measured rate | 9.92 Hz |
| Depth while stationary in air | 33 to 45 mm |
| Roll while stationary | 1.67 to 1.69 degrees |
| Pitch while stationary | 0.74 to 0.76 degrees |
| Yaw while stationary | 276.66 to 277.17 degrees |
| Status | `0x0F1C` |

The continuous sequence numbers and increasing uptime verify stable framing.
The narrow stationary attitude range shows that the live sensor data is not a
fixed placeholder.

### 4.4 Yaw integration

Earlier live runs produced stable headings near 282 degrees, with heading
changes through approximately 276 to 286 degrees when the installed assembly
was reoriented. Status `0x0F18` confirmed that the IMU, magnetometer, and depth
sensor were all detected and that depth, roll/pitch, and yaw were valid.

This proves live magnetometer acquisition and tilt-compensated yaw. Absolute
heading accuracy is not yet claimed because installed-system hard-iron and
soft-iron calibration has not been performed.

### 4.5 Link-loss failsafe

Command:

```text
python3 ~/uart_failsafe_test.py /dev/ttyUSB0
```

Observed result:

```text
PASS: COMMAND_LINK_OK=1 and FAILSAFE_ACTIVE=0 while commands arrive
Failsafe reported after 0.511 s: status=0x0F1C, armed=0, last_cmd=83
PASS: link loss cleared arm and activated neutral failsafe
```

The STM32 accepted valid commands at 10 Hz. When the Jetson stopped sending,
the STM32 independently detected the stale link, cleared arm, atomically
replaced the active command with zero motion/depth setpoints, and reported
failsafe after 0.511 seconds.

### 4.6 Individual thruster mapping and PWM output

The neutral PWM test verified 50 Hz timer configuration, enabled timer
channels, and a 1500 us compare value on all eight outputs. With the ROV
secured and the propulsion battery connected, all thrusters remained stopped
at neutral. Each output was then pulsed individually for 250 ms at low power.

| Command | Thruster | Physical result |
|---|---|---|
| `M1+` | `V_FL` | Rotated and stopped |
| `M2+` | `V_FR` | Rotated and stopped |
| `M3+` | `V_RL` | Rotated and stopped |
| `M4+` | `V_RR` | Rotated and stopped |
| `M5+` | `O_FL` | Rotated and stopped |
| `M6+` | `O_FR` | Rotated and stopped |
| `M7+` | `O_RL` | Rotated and stopped |
| `M8+` | `O_RR` | Rotated and stopped |

This confirms the STM32 timer/channel-to-thruster map used by the mixer.

### 4.7 Integrated command, mixer, sensors, E-stop, and failsafe

Command:

```text
python3 ~/uart_motor_integration_test.py /dev/ttyUSB0
```

The test armed at neutral, checked that depth/roll/pitch/yaw remained valid,
and exercised `HEAVE`, `SURGE`, `SWAY`, and `YAW` through the real binary
command and mixer path. It inserted neutral pauses between axes, asserted an
Emergency Stop, re-armed at neutral, and finally stopped UART commands while
heave was requested.

Observed automated result:

```text
PASS: all four mixer axes completed with healthy telemetry
PASS: Emergency Stop cleared ARMED and forced neutral
PASS: link loss forced neutral after 0.457 s
PASS: integrated commands, sensors, mixer, E-stop, and failsafe work
Final state requested: EMERGENCY STOP / neutral
```

The firmware remained inside its low-power PWM clamp for every phase. Visual
confirmation of the grouped `HEAVE`, `SURGE`, `SWAY`, and `YAW` motions and
their physical stopping response is recorded separately by the operator; the
automated test validates packets, status, timing, and the commanded PWM path.

## 5. Status word interpretation

`0x0F18` means:

- bit 3: depth valid;
- bit 4: roll/pitch valid;
- bit 8: MPU6050 detected;
- bit 9: MS5837 detected;
- bit 10: yaw valid;
- bit 11: magnetometer detected.

`0x0F1C` contains the same valid sensor bits plus bit 2,
`FAILSAFE_ACTIVE`. `COMMAND_LINK_OK` is clear because continuous commands have
stopped. `ARMED` is also clear.

## 6. Task acceptance summary

| Task | Status | Evidence |
|---|---|---|
| Define packet protocol and control interface | Complete | Protocol specification, encoder/parser tests, mutex-protected shared snapshots |
| Build UART receive path | Complete | Five real-UART command cases passed |
| Build telemetry transmit path | Complete | 20 valid live packets at 9.92 Hz with zero CRC/header errors |
| Integrate real sensors | Complete for acquisition | Live MPU6050, magnetometer yaw, and MS5837 depth |
| Motor mixer and output mapping | Complete | Mixer self-test plus eight individual physical thruster tests |
| Link-loss failsafe | Commanded-output path complete | 0.457 s integrated transition, arm cleared, neutral applied |
| Integration and testing | Automated test complete | Commands, sensors, four mixer axes, E-stop, and link loss ran together |
| Grouped physical-motion confirmation | Awaiting operator observation | Confirm each group moved and stopped as labelled during the integrated run |

## 7. Remaining work before unrestricted propulsion

1. Calibrate the magnetometer in its final installed position to compensate
   for power wiring, regulators, steel, and other magnetic interference.
2. Validate and calibrate MS5837 depth in water; the present zero is the
   average of the first 20 valid samples after boot.
3. Record visual confirmation that the grouped `HEAVE`, `SURGE`, `SWAY`, and
   `YAW` responses matched their labels and stopped during both stop tests.
4. Verify every thruster's physical sign and the vehicle-level mixer geometry
   in water; dry-air rotation alone does not validate hydrodynamic direction.
5. Integrate the final PID/control outputs, then repeat E-stop and link-loss
   tests under the controlled in-water test procedure.
6. Keep the `+/- 60 us` validation clamp until direction, current draw, and
   vehicle restraint are verified; removing it is a separate production step.

## 8. Conclusion

The STM32F411–Jetson stack now operates end to end through the real low-power
motor output path. Binary framing, CRC validation, malformed-frame recovery,
live sensor telemetry, all eight physical PWM mappings, four-axis mixing,
Emergency Stop, and the independent 500 ms link-loss failsafe passed. The
present firmware is an intentionally power-limited validation build, not yet
an unrestricted propulsion release.
