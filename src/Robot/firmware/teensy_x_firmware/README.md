# Teensy 4.1 Serial I/O Coprocessor Firmware

This firmware targets a **Teensy 4.1** and is meant to be the higher-capacity replacement for the Nano Every coprocessor sketch in this repo.

It supports:
- 4 PWM + DIR drive motors
- 3 PWM + DIR mechanism motors
- goBILDA / FTC-style quadrature motor encoders
- 2 relay outputs
- 1 RGBW LED group
- Up to 3 hobby servos
- JSON serial control + telemetry
- Motor watchdog timeout

## File
- `teensy_x_firmware.ino`

## Protocol
The sketch uses the same JSON line protocol as the Nano coprocessor:
- `{"type":"ping"}`
- `{"type":"mode","mode":"TELEOP"}`
- `{"type":"drive","motors":[0.2,-0.2,0.2,-0.2]}`
- `{"type":"mech","motors":[0.5,0.0,-0.3]}`
- `{"type":"relay","index":0,"on":true}`
- `{"type":"led","index":0,"r":255,"g":64,"b":0,"w":0}`
- `{"type":"servo","channel":1,"value":0.0}`
- `{"type":"servo_detach","channel":1}`
- `{"type":"reset"}`

Servo values are in `[-1.0, 1.0]`.

## Wiring Notes
- Update the pin maps at the top of the sketch before flashing.
- Keep motor PWM pins on Teensy PWM-capable pins.
- Encoder pins are attached with interrupts on both A and B for full quadrature counting.
- Servos need an external 5V rail with common ground to the Teensy.
- If an encoder or motor runs backward, flip `invert` / `invertDir` in the config instead of rewiring everything.

## Telemetry
Periodic telemetry includes:
- encoder counts
- relay states
- servo command values
- LED values
- drive and mechanism motor commands
- robot mode / watchdog state
