# Autonomous Support

The robot server now includes a lightweight autonomous runner that can:

- drive set distances relative to the robot's current heading
- turn to set field angles
- drive to a target field pose
- run multi-step routines from a simple step list

## Command types

Send these through the same command socket as the other robot commands.

### One-shot helpers

`auto_drive_distance`

```json
{
  "type": "auto_drive_distance",
  "forward_m": 1.0,
  "strafe_m": 0.0,
  "hold_heading_deg": 90.0,
  "speed": 0.35,
  "timeout_s": 5.0
}
```

Notes:

- `distance_m` can be used as an alias for `forward_m`
- `strafe_m` is optional
- `hold_heading_deg` defaults to the heading at the start of the move

`auto_turn_to_angle`

```json
{
  "type": "auto_turn_to_angle",
  "angle_deg": 180.0,
  "speed": 0.3,
  "timeout_s": 4.0
}
```

`auto_drive_to_pose`

```json
{
  "type": "auto_drive_to_pose",
  "x_m": 1.5,
  "y_m": 0.8,
  "heading_deg": 180.0,
  "speed": 0.35,
  "turn_speed": 0.25,
  "timeout_s": 8.0
}
```

### Routine control

`auto_run`

```json
{
  "type": "auto_run",
  "routine": {
    "name": "score_and_back_up",
    "steps": [
      {"action": "set_odometry_mode", "mode": "OPTICAL"},
      {"action": "drive_distance", "forward_m": 1.0, "speed": 0.35, "timeout_s": 5.0},
      {"action": "turn_to_angle", "angle_deg": 180.0, "speed": 0.25, "timeout_s": 4.0},
      {"action": "wait", "duration_s": 0.5},
      {"action": "drive_distance", "forward_m": 0.5, "speed": 0.3, "timeout_s": 4.0}
    ]
  }
}
```

`auto_status`

```json
{"type": "auto_status"}
```

`auto_cancel`

```json
{"type": "auto_cancel"}
```

## Supported step actions

`wait`

- `duration_s`

`drive_distance`

- `forward_m`
- `strafe_m`
- `hold_heading_deg`
- `speed`
- `turn_speed`
- `timeout_s`
- `position_tolerance_m`
- `angle_tolerance_deg`

`turn_to_angle`

- `angle_deg`
- `speed`
- `timeout_s`
- `angle_tolerance_deg`

`drive_to_pose`

- `x_m`
- `y_m`
- `heading_deg`
- `speed`
- `turn_speed`
- `timeout_s`
- `position_tolerance_m`
- `angle_tolerance_deg`

`set_odometry_mode`

- `mode`

`reset_odometry`

- no extra fields

## Behavior notes

- Starting an auto routine puts the robot in `AUTO` mode automatically.
- Leaving `AUTO` mode cancels the active routine.
- A communication-loss estop also stops the active routine.
- Motion uses the robot's current pose estimate, so `OPTICAL` or `HYBRID` odometry will give better distance and angle accuracy than dead-reckoning alone.
- Control tuning values live in `constants.py`.
