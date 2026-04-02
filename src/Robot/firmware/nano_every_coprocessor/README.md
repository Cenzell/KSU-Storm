# Nano Every Serial I/O Coprocessor Firmware

This firmware lets an **Arduino Nano Every** act as an I/O coprocessor for a Raspberry Pi over USB serial.

It supports:
- PWM + DIR motor outputs
- Quadrature encoder counting
- Servo outputs via Adafruit PCA9685 (`Adafruit_PWMServoDriver`)
- Relay outputs
- Motor watchdog timeout
- Periodic telemetry streaming

## File
- `nano_every_coprocessor.ino`

## Default Pin Map (edit in sketch to match your robot)
- Motors:
  - Motor 0: `PWM D9`, `DIR D4`
  - Motor 1: `PWM D10`, `DIR D7`
- Encoders:
  - Encoder 0: `A D2`, `B D12`
  - Encoder 1: `A D3`, `B D13`
- Relays:
  - Relay 0: `D8`
- Servos (PCA9685 channels):
  - Servo 0: `CH0`
  - Servo 1: `CH1`

## Serial Settings
- Baud: `115200`
- Line format: ASCII CSV commands ending with `\n`

On boot it prints:
- `READY,NANO_EVERY_IO_V1`

## Command Protocol
- `PING`
  - Response: `PONG`

- `M,<id>,<signed_pwm>`
  - `id`: motor index
  - `signed_pwm`: `-255..255`
  - Response: `OK`

- `MD,<id>,<duty>,<dir>`
  - `duty`: `0..255`
  - `dir`: `0` reverse, `1` forward
  - Response: `OK`

- `R,<id>,<0|1>`
  - Relay control
  - Response: `OK`

- `S,<id>,<microseconds>`
  - Servo pulse width (`500..2500` us) on PCA9685 channel map
  - Response: `OK`

- `SP,<id>,<degrees>`
  - Servo angle (`0..180`)
  - Response: `OK`

- `SRVDETACH,<id>`
  - Disable PCA9685 output for that servo channel
  - Response: `OK`

- `Q,ENC`
  - Response: `ENC,<enc0>,<enc1>,...`

- `Q,STAT`
  - Response:
  - `STAT,<uptime_ms>,<ms_since_cmd>,<m0>,<m1>,...,<r0>,...,<enc0>,<enc1>,...`

- `E,RESET`
  - Reset all encoder counts to zero
  - Response: `OK`

- `W,<timeout_ms>`
  - Motor watchdog timeout (`50..5000` ms)
  - Response: `OK`

- `T,<rate_hz>`
  - Telemetry stream rate (`0..100` Hz), `0` disables streaming
  - Response: `OK`

Errors return:
- `ERR,<reason>`

## Watchdog Behavior
If no valid command is received within timeout, all motors are forced to `0` PWM.

## PCA9685 Requirement
- Install Arduino library: `Adafruit PWM Servo Driver Library`
- Connect PCA9685 to Nano Every I2C (`SDA`, `SCL`, `5V`, `GND`)
- Servos should be powered from a proper 5V rail (not Nano USB power)

## Quick Raspberry Pi Serial Test
```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.2)
time.sleep(2.0)  # let Nano reboot after open

print(ser.readline().decode(errors='ignore').strip())  # READY...

for cmd in [
    'PING\n',
    'M,0,120\n',
    'M,1,-120\n',
    'R,0,1\n',
    'Q,ENC\n',
    'Q,STAT\n',
    'R,0,0\n',
    'M,0,0\n',
    'M,1,0\n',
]:
    ser.write(cmd.encode())
    print('>', cmd.strip())
    print('<', ser.readline().decode(errors='ignore').strip())
```

## Notes for goBILDA Encoders
- This template uses x1 decoding on A-edge changes for reliability and lower ISR load.
- If direction is reversed, swap A/B for that encoder or invert sign in software.
- For very high RPM/high CPR cases, you may want a dedicated encoder IC or optimized interrupt strategy.
