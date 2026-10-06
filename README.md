# STM32F411–Jetson ROV Controller

This repository contains the low-level controller and communication link for
our student ROV project. I use an **STM32F411CEU6 BlackPill** for real-time
sensor reading, safety logic, motor mixing, and PWM generation. An
**NVIDIA Jetson** sends motion commands and receives telemetry through a UART
link.

The purpose of this stage was not to claim that the complete ROV is ready for
the water. My goal was to build and prove the full low-level path first:

```text
Jetson command -> UART packet -> STM32 parser -> safety check
               -> motor mixer -> 8 PWM outputs

STM32 sensors -> telemetry packet -> UART -> Jetson test programs
```

## Current project status

The BlackPill–Jetson communication tasks are complete and tested on the real
hardware.

| Part | Status | Evidence |
|---|---|---|
| Binary packet protocol and CRC-16 | Complete | Encoder/parser self-tests passed |
| UART receive path | Complete | Good, noisy, partial, bad-CRC, bad-length, and out-of-range cases tested |
| Telemetry transmit path | Complete | 20 valid packets at about 9.9 Hz with zero CRC/header errors |
| Live sensors | Complete for acquisition | MPU6050 roll/pitch, HW-290 magnetometer yaw, and MS5837 depth |
| Eight-thruster PWM mapping | Complete | `M1` through `M8` physically tested one at a time |
| Four-axis mixer | Complete for bench validation | Heave, surge, sway, and yaw command paths passed |
| Emergency Stop | Complete | Cleared armed state and forced neutral |
| UART link-loss failsafe | Complete | Repeated transitions to neutral in 0.457–0.459 s |
| Water commissioning | Not started | Direction, buoyancy, and depth accuracy still need an in-water test |
| Closed-loop PID/depth hold | Not finished | Planned after water commissioning |

The current firmware is a **low-power validation build**. Motor outputs are
enabled, but [`app_mode.h`](Core/Inc/app_mode.h) limits every channel to
`1500 +/- 60 us`. This limit must remain in place until the ROV is restrained
and tested safely in water.

## Hardware used

- STM32F411CEU6 BlackPill
- NVIDIA Jetson running Ubuntu 24.04
- CH340 USB-to-TTL adapter
- MPU6050 accelerometer/gyroscope
- HW-290 companion QMC5883L/HMC5883L magnetometer
- MS5837 pressure/depth sensor
- Eight bidirectional ESC/thruster channels
- ST-Link for programming and debugging

The STM32 runs at **100 MHz**. FreeRTOS separates UART reception, telemetry,
sensor acquisition, LED heartbeat, and control/safety work.

## Important wiring

### Jetson UART

The motherboard may label this connection as UART1, but the firmware uses
STM32 **USART6**:

| USB-TTL | STM32F411 | Function |
|---|---|---|
| TX | PA12 | USART6 RX |
| RX | PA11 | USART6 TX |
| GND | GND | Common reference |
| VCC | Not connected | The ROV powers the BlackPill separately |

Wire colours are not standardized. Always follow the `TX`, `RX`, `GND`, and
`VCC` labels printed on the adapter.

### Sensors

| Sensor | Bus | STM32 pins |
|---|---|---|
| MPU6050 + HW-290 magnetometer | I2C1 | PB6 SCL, PB7 SDA |
| MS5837 depth sensor | I2C3 | PA8 SCL, PB4 SDA |

### Thruster outputs

| Motor | Position | Timer channel | Pin |
|---|---|---|---|
| M1 | `V_FL` vertical front-left | TIM4 CH3 | PB8 |
| M2 | `V_FR` vertical front-right | TIM4 CH4 | PB9 |
| M3 | `V_RL` vertical rear-left | TIM2 CH1 | PA0 |
| M4 | `V_RR` vertical rear-right | TIM3 CH4 | PB1 |
| M5 | `O_FL` horizontal front-left | TIM3 CH1 | PA6 |
| M6 | `O_FR` horizontal front-right | TIM2 CH2 | PA1 |
| M7 | `O_RL` horizontal rear-left | TIM3 CH2 | PA7 |
| M8 | `O_RR` horizontal rear-right | TIM3 CH3 | PB0 |

PWM is 50 Hz. Neutral is `1500 us`; the configured full software range is
`1200–1800 us`, but the current validation build clamps the actual hardware
output to `1440–1560 us`.

## Motion commands

The Jetson command packet carries four manual motion values from `-1000` to
`+1000`:

- **Heave:** vertical movement using all four vertical thrusters.
- **Surge:** forward/backward movement using all four horizontal thrusters.
- **Sway:** left/right movement using the horizontal diagonal mix.
- **Yaw:** rotation around the vertical axis using opposing horizontal pairs.

The sign of each vehicle movement still needs to be confirmed in water. A
thruster spinning on the bench proves its electrical mapping, but not the
final hydrodynamic direction of the complete vehicle.

## Communication and safety

UART settings are `115200 baud, 8 data bits, no parity, 1 stop bit`.
Commands and telemetry use a versioned binary frame with:

- two start bytes (`AA 55`);
- message type and payload length;
- an 8-bit sequence number;
- little-endian payload fields;
- CRC-16/CCITT-FALSE.

The full byte-level definition is in
[`Docs/packet_protocol_spec.md`](Docs/packet_protocol_spec.md).

The STM32—not the Jetson—owns the final safety decision:

1. Invalid packets never update the active command.
2. A valid armed command must keep arriving before the 500 ms timeout.
3. `EMERGENCY_STOP` immediately clears arm and requests neutral PWM.
4. If the UART command stream stops, the STM32 independently returns every
   motor output to neutral.
5. The integration script also sends Emergency Stop in its `finally` block.

## Repository layout

```text
Core/          STM32 application, drivers, parser, mixer, and FreeRTOS tasks
Drivers/       STM32 HAL and CMSIS sources generated with STM32CubeIDE
Middlewares/   FreeRTOS middleware
Jetson/        Python UART tests that run on the Jetson
Docs/          Protocol specification, test report, and engineering notes
*.ioc          STM32CubeMX hardware configuration
```

Generated `Debug/`, Python cache files, and local debugger launch files are
excluded from Git.

## Build and flash

1. Open `stm32_rov_working.ioc` or import the project in STM32CubeIDE.
2. Select the `Debug` configuration.
3. Build the project.
4. Connect ST-Link using SWD and flash the generated ELF.
5. Confirm the BlackPill LED blinks before connecting propulsion power.

Latest recorded build:

```text
text     data     bss      dec      hex
75120    108      22128    97356    17c4c
Build Finished: 0 errors, 1 warning
```

The remaining warning is the linker message that the ELF has an RWX LOAD
segment. It did not prevent flashing or execution, but the linker script
permissions should be cleaned up later.

## Jetson setup and tests

Install the Python dependency:

```bash
python3 -m pip install -r Jetson/requirements.txt
```

The CH340 adapter must appear as `/dev/ttyUSB0` or another serial device. Then
the main tests are:

```bash
python3 Jetson/uart_ping_test.py /dev/ttyUSB0
python3 Jetson/uart_parser_selftest.py /dev/ttyUSB0
python3 Jetson/uart_command_test.py /dev/ttyUSB0
python3 Jetson/uart_telemetry_test.py /dev/ttyUSB0
python3 Jetson/uart_failsafe_test.py /dev/ttyUSB0
python3 Jetson/uart_motor_integration_test.py /dev/ttyUSB0
```

The motor integration test requires typing `RUN` before it sends an armed
motion command. Secure the ROV, keep people and loose objects away from all
propellers, and keep a physical power disconnect within immediate reach.

Detailed measured results are recorded in
[`Docs/integration_test_report.md`](Docs/integration_test_report.md).

## What I learned from this stage

This project taught me that communication working once is not enough. I had
to test corrupted packets, partial packets, sensor data and binary telemetry
on the same UART, motor mapping, neutral PWM, Emergency Stop, and loss of the
Jetson link. The most important design decision was keeping the failsafe on
the STM32 so the motors can stop even when the Jetson or UART connection fails.

## Next steps

1. Leak-test the enclosure with power off.
2. Put the restrained ROV in shallow water with the `+/- 60 us` clamp active.
3. Confirm depth, roll, pitch, and yaw while the vehicle is in the water.
4. Confirm the physical sign of heave, surge, sway, and yaw; correct mixer
   signs if necessary.
5. Calibrate the magnetometer in its final installed position.
6. Validate the MS5837 depth conversion against measured depth.
7. Add and tune closed-loop depth/attitude control.
8. Repeat Emergency Stop and link-loss tests before increasing thrust.

## Author

Developed and tested by **Ahmed** 

