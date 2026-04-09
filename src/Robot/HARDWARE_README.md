# KSU Storm Hardware I/O Map

This document summarizes the hardware I/O currently defined in code for the robot.

Current hardware intent:

- All motors are controlled by the Teensy 4.1
- The Raspberry Pi handles high-level robot code, USB serial to the Teensy, and Pi-side sensors
- A PCA9685 should not be used for motor control

## Active communication paths

### Raspberry Pi to Teensy

- Connection type: USB serial
- Default device: `/dev/ttyACM0`
- Baud rate: `115200`
- Code path: `lib/serial_bridge.py`

This is not mapped to Pi header GPIO pins in the repo; it is expected to be a USB connection.

### Raspberry Pi I2C devices

- SparkFun Qwiic OTOS optical odometry sensor
  - Bus: Qwiic / I2C on the Raspberry Pi
  - Used by `hardware/optical_odometry_sensor.py`

## Raspberry Pi connections

Pi GPIO references in this document use BCM numbering when applicable.

### Pi-side devices and buses

| Device | Pi connection | Software name | Purpose |
|---|---|---|---|
| Teensy 4.1 | USB serial `/dev/ttyACM0` | — | Main MCU coprocessor link |
| Qwiic OTOS | I2C `SDA` / `SCL` | — | Optical odometry |
| Pi Camera | CSI ribbon connector 0 | `driver` | **Primary driver view** — picamera2 backend, AprilTag off |
| USB Camera 1 | USB (OpenCV index `0`) | `front_left` | Secondary view — AprilTag detection enabled |
| USB Camera 2 | USB (OpenCV index `1`) | `front_right` | Tertiary view — AprilTag detection enabled |

#### Camera notes

- Override the USB camera OpenCV indices with env vars `KSU_CAMERA_FRONT_LEFT_SELECTOR` and `KSU_CAMERA_FRONT_RIGHT_SELECTOR` if the system assigns them differently (run `v4l2-ctl --list-devices` to confirm).
- If only one USB camera is connected, the second will fail gracefully after `CAMERA_INIT_MAX_ATTEMPTS` retries and simply not stream.
- AprilTag detection on `driver` is disabled by default — enable via `KSU_CAMERA_DRIVER_ENABLE_APRILTAG=1` if needed.

## Teensy 4.1 pin map

This section reflects the pin arrays currently defined in `src/Robot/firmware/teensy_x_firmware/teensy_x_firmware.ino`.

### Drive motors

Motor order is `[FL, FR, RL, RR]`.

| Motor | PWM pin | DIR pin | invertDir |
|---|---:|---:|---|
| Front Left | `2` | `22` | `false` |
| Front Right | `3` | `23` | `false` |
| Rear Left | `4` | `28` | `false` |
| Rear Right | `5` | `29` | `false` |

### Mechanism motors

| Mechanism motor | PWM pin | DIR pin | invertDir |
|---|---:|---:|---|
| Elevator Left | `6` | `30` | `false` |
| Elevator Right | `7` | `31` | `false` |
| Arm Motor | `8` | `32` | `false` |

### Relays

| Relay index | Pin | activeHigh |
|---|---:|---|
| Relay 0 | `33` | `true` |
| Relay 1 | `34` | `true` |

### Signal Light LED (§4.4.8)

LED Group 0 is reserved for the **competition signal light** required by rule §4.4.8.
It is driven automatically by `signal_light_thread` in `src/Robot/robot.py` — do not use it for anything else.

| LED group | Purpose | R pin | G pin | B pin | W pin | commonAnode |
|---|---|---:|---:|---:|---:|---|
| LED 0 | Signal light (§4.4.8) | `9` | `10` | `11` | `12` (unused) | `false` |

#### Behaviour

| Robot state | LED colour | Pattern |
|---|---|---|
| Connected (has signal) | Green | Blink ~1 Hz (0.5 s on / 0.5 s off) |
| Loss of Signal / stopped | Red | Solid (no blink) |

#### Wiring a 4-pin common-cathode RGB LED

```
Teensy 4.1          Resistor    LED pin
─────────────────────────────────────────
Pin  9  (PWM)  ──►  150 Ω  ──►  R  (red)
Pin 10  (PWM)  ──►  100 Ω  ──►  G  (green)
Pin 11  (PWM)  ──►  100 Ω  ──►  B  (blue)
GND            ─────────────►  GND (cathode)
```

Pin 12 (W channel) is defined in the firmware struct but **not connected** for a 3-channel RGB LED.

**Resistor selection** (Teensy 4.1 runs at 3.3 V, GPIO rated to 3.3 V / ~8 mA per pin):

| Channel | Vf (typ) | Formula | Chosen value |
|---|---|---|---|
| Red | 2.0 V | (3.3 − 2.0) / 0.010 = 130 Ω | **150 Ω** |
| Green | 3.0 V | (3.3 − 3.0) / 0.010 = 30 Ω | **100 Ω** (limits current, protects pin) |
| Blue | 3.0 V | (3.3 − 3.0) / 0.010 = 30 Ω | **100 Ω** |

Use the larger value when in doubt — the LED will still be clearly visible and the pin stays within its 8 mA limit.

> **Common-anode variant?** Flip the shared pin to 3.3 V, connect each colour pin to the resistor/Teensy as above, and change `commonAnode` to `true` in the firmware `LedGroupConfig`. The firmware already inverts the PWM value when `commonAnode = true`.

#### Why pins 9 / 10 / 11?

All three are PWM-capable on the Teensy 4.1 and are not used by any other subsystem in this repo.  
Pins 11 and 12 overlap with the SPI0 MOSI/MISO signals, but SPI0 is not used here — they are free to use as general PWM outputs.

### Servos

| Servo channel | Pin | Min pulse (us) | Max pulse (us) |
|---|---:|---:|---:|
| Servo 0 | `35` | `500` | `2500` |
| Servo 1 | `36` | `500` | `2500` |
| Servo 2 | `37` | `500` | `2500` |

### Encoders

Encoder slots match the total motor count: 4 drive + 3 mechanism.

| Encoder index | Expected device | Pin A | Pin B | invert |
|---|---|---:|---:|---|
| 0 | Drive motor FL | `14` | `15` | `false` |
| 1 | Drive motor FR | `16` | `17` | `false` |
| 2 | Drive motor RL | `18` | `19` | `false` |
| 3 | Drive motor RR | `20` | `21` | `false` |
| 4 | Elevator Left | `24` | `25` | `false` |
| 5 | Elevator Right | `26` | `27` | `false` |
| 6 | Arm Motor | `38` | `39` | `false` |

## What is actually used today

Based on the current Python robot server:

- Drive commands are always generated in software as four wheel outputs
- Drive and mechanism commands are intended to be sent to the Teensy
- Optical odometry is read from a Qwiic OTOS sensor on the Pi side

## Legacy code note

Some older Raspberry Pi direct-motor and PCA9685 motor-control code still exists in the repo, but that is not the intended hardware setup anymore. The intended wiring is:

- all drive motors on the Teensy
- all mechanism motors on the Teensy
- no PCA9685 motor control

## Not currently mapped in the repo

The repo does not currently define header-pin numbers for:

- Raspberry Pi USB connection to the Teensy
- Camera ribbon or USB camera wiring
- Power distribution wiring
- Motor controller screw-terminal wiring
- Exact relay load wiring

Those pieces would need to be added manually if you want a full electrical wiring guide instead of a software-defined I/O map.
