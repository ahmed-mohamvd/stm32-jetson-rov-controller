# Jetson to STM32 UART smoke test

This is the first communication milestone. It does not use the motors, ESCs,
IMU, or depth sensor.

## Runtime wiring

The firmware uses `USART6` at `115200 8N1`. The robot motherboard may label
this physical connection as `UART1`, but the STM32 peripheral on PA11/PA12 is
USART6:

- STM32 `PA11` (`USART6_TX`) -> USB-TTL/Jetson `RX`
- STM32 `PA12` (`USART6_RX`) <- USB-TTL/Jetson `TX`
- STM32 `GND` <-> USB-TTL `GND`
- Leave the USB-TTL power/VCC wire disconnected when the Blackpill has its own
  USB power.

Use the labels printed on the USB-TTL adapter, not wire colours, because colours
are not standardized.

For normal firmware execution, set `BOOT0` low (`0`) and reset/power-cycle the
Blackpill. The ROM bootloader used for flashing and the application UART are
different operating modes.

## Run on the Jetson

Plug the USB-TTL adapter into the Jetson, then run:

```bash
sudo apt install python3-serial
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
python3 uart_ping_test.py /dev/ttyUSB0
```

If the adapter appears with a different name, pass that name instead of
`/dev/ttyUSB0`.

Expected result:

```text
Jetson -> STM32: PING
STM32 -> Jetson: PONG
PASS: bidirectional UART communication works
```
