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

| Device | Pi connection | Purpose |
|---|---|---|
| Teensy 4.1 | USB serial on `/dev/ttyACM0` | Main MCU coprocessor link |
| Qwiic OTOS | I2C `SDA` / `SCL` | Optical odometry |

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

### LED group

| LED group | R pin | G pin | B pin | W pin | commonAnode |
|---|---:|---:|---:|---:|---|
| LED 0 | `9` | `10` | `11` | `12` | `false` |

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
