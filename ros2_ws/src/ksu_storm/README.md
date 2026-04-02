# KSU Storm ROS 2 Port

This package is the first ROS 2 Jazzy migration step for the KSU Storm robot.

## Current scope

- Runs the robot runtime as an `rclpy` node
- Subscribes to `cmd_vel` (`geometry_msgs/Twist`)
- Subscribes to `robot_mode` (`std_msgs/String`)
- Subscribes to `odometry_mode` (`std_msgs/String`)
- Publishes `odom` (`nav_msgs/Odometry`)
- Publishes `telemetry/json` (`std_msgs/String`)
- Publishes `diagnostics` (`diagnostic_msgs/DiagnosticArray`)
- Provides `reset_robot` (`std_srvs/Trigger`)
- Provides `reset_odometry` (`std_srvs/Trigger`)

## Build

From the repository root:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

If you build without `--symlink-install`, set:

```bash
export KSU_STORM_REPO_ROOT=/home/cenzell/PythonProjects/KSU-Storm
```

## Run

```bash
ros2 launch ksu_storm robot.launch.py
```

## Notes

- The existing ZMQ-based driver station is still untouched.
- Camera streaming is not ROS-native yet.
- The node still reuses the existing serial bridge and hardware helpers from the main repo.
