# Student engineering notes

These notes describe what I implemented, the problems I found, and what the
current test results really prove. I am keeping this separate from the formal
integration report so the project history is easier to understand.

## 1. Starting point

The STM32 code could build and flash, but the original ST-Link message often
said that no target was found. After checking the SWD connection and flashing
again, the STM32F411CEU6 was programmed successfully and the heartbeat LED
confirmed that the application was running.

The first goal was communication between the Jetson and the BlackPill. I used
a CH340 USB-to-TTL adapter with crossed TX/RX and a common ground. VCC was not
connected because the STM32 was already powered by the ROV electronics.

The Jetson kernel did not include the CH341 module. I built `ch341.ko` against
the matching `6.8.12-1021-tegra` headers, loaded it, installed it under
`/lib/modules`, ran `depmod`, and added the user to `dialout`. The adapter then
appeared as `/dev/ttyUSB0`.

## 2. Communication milestones

The first simple test sent the ASCII word `PING` and received `PONG`. After
that, I replaced the simple message with a real binary protocol. The protocol
has framing bytes, version, type, payload length, sequence number, and CRC.

I tested more than the good path:

- random noise before a valid command;
- a command with a corrupted CRC;
- invalid payload length;
- a partial packet followed by timeout and recovery;
- motion values outside the allowed range.

The STM32 accepted the valid commands and rejected the malformed ones without
locking up or refreshing the safety timer.

## 3. Real telemetry

At first the telemetry values were placeholders. I then connected the actual
sensor readers:

- MPU6050 for roll and pitch;
- QMC5883L/HMC5883L on the HW-290 module for tilt-compensated yaw;
- MS5837 pressure sensor for relative depth.

The Jetson received telemetry at about 9.9 Hz. Twenty-packet tests completed
with no CRC or header errors. The readings changed when the hardware moved,
which confirmed that the data was live rather than a fixed example.

Yaw is live but not fully calibrated. Metal, high-current wiring, and the
installed electronics can change the magnetic field, so final hard-iron and
soft-iron calibration must be done with the complete ROV assembled.

## 4. Motor mapping and mixer

I first locked all eight PWM outputs at 1500 us and checked that every motor
stayed stopped. Then I tested the motors individually for 250 ms using
`M1+` through `M8+`. This confirmed all eight timer-channel mappings.

After individual testing, I enabled the real binary-command path and used the
four-axis mixer:

- heave drives the four vertical thrusters;
- surge drives the four horizontal thrusters in the same command direction;
- sway drives diagonal pairs in opposite directions;
- yaw drives left/right turning pairs in opposite directions.

For bench testing I added a final PWM clamp of `1500 +/- 60 us`. This is a
hardware boundary after the mixer, so even a full-scale valid command cannot
produce more than the low-power validation output.

## 5. Safety tests

The 500 ms command watchdog is implemented on the STM32. It does not depend on
the Jetson asking the vehicle to stop.

The integrated test performed these stages:

1. armed at neutral while checking live depth, roll, pitch, and yaw flags;
2. low-power heave, surge, sway, and yaw with neutral pauses;
3. Emergency Stop while a motion command was active;
4. re-arm at neutral;
5. active heave followed by deliberately stopping UART commands.

Three complete integration runs passed. The measured link-loss transitions
were `0.457 s`, `0.459 s`, and `0.458 s`. Every run finished by requesting
Emergency Stop and neutral.

## 6. What is complete

The BlackPill–Jetson communication task is complete at bench-validation level:

- protocol and shared interface;
- UART receive parser;
- telemetry transmitter;
- real sensors;
- eight PWM mappings;
- four-axis open-loop mixer;
- Emergency Stop;
- independent link-loss failsafe;
- repeated end-to-end integration tests.

## 7. What is not complete

I should not describe the entire ROV as finished yet. The following work still
needs to be done:

- leak and immersion tests;
- confirmation of movement directions in water;
- accurate depth calibration;
- installed magnetometer calibration;
- buoyancy and trim adjustment;
- PID/depth-hold implementation and tuning;
- current and full-power propulsion validation.

The present result proves the low-level communication and safety architecture.
It does not yet prove autonomous control or full vehicle performance in water.

