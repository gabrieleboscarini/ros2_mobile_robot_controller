# ros2_mobile_robot_controller

ROS 2 (Jazzy) workspace for driving and safety-monitoring a simulated differential-drive
robot in Gazebo Harmonic. Built on top of the [bme_gazebo_sensors](https://github.com/CarmineD8/bme_gazebo_sensors)
simulation package.

## Packages

| Package | Type | Description |
|---|---|---|
| `bme_gazebo_sensors` | ament_cmake | Gazebo world, robot URDF, sensor plugins (lidar, diff-drive), launch files |
| `robot_controller` | ament_python | `driver_node` and `safety_node` |
| `robot_interfaces` | ament_cmake | Custom service/message definitions used by `robot_controller` |

## Requirements

- ROS 2 Jazzy
- Gazebo Harmonic
- `ros_gz_bridge`, `ros_gz_sim`, `rviz2` (standard ROS 2 Jazzy packages)

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

**Terminal 1 — simulation (Gazebo + RViz + robot):**
```bash
source install/setup.bash
ros2 launch bme_gazebo_sensors spawn_robot.launch.py
```

**Terminal 2 — driver node** (publishes `/cmd_vel`, exposes velocity control):
```bash
source install/setup.bash
ros2 run robot_controller driver_node
```

**Terminal 3 — safety node** (monitors `/scan`, publishes obstacle info):
```bash
source install/setup.bash
ros2 run robot_controller safety_node
```

**Drive the robot:**
```bash
ros2 service call /set_velocity robot_interfaces/srv/SetVelocity "{linear: 0.3, angular: 0.2}"
```

**Change the safety distance threshold:**
```bash
ros2 service call /set_threshold robot_interfaces/srv/SetThreshold "{threshold: 0.8}"
```

**Read the average of the last 5 velocity commands:**
```bash
ros2 service call /get_average_velocity robot_interfaces/srv/GetAverageVelocity "{}"
```

**Watch obstacle info** (closest obstacle distance, direction, current threshold):
```bash
ros2 topic echo /obstacle_info
```

## Behavior

- `driver_node` owns `/cmd_vel` and republishes the last commanded velocity at 10 Hz.
- `safety_node` reads `/scan`, computes the distance and direction (`front`/`left`/`right`)
  of the closest obstacle, and publishes it on `/obstacle_info` together with the current
  threshold.
- When `driver_node` receives an `ObstacleInfo` message with `distance < threshold`, it
  reverses the current velocity (negates linear and angular) and keeps publishing the
  reversed command until the user issues a new `/set_velocity` call.

## Custom interfaces (`robot_interfaces`)

| Interface | Fields |
|---|---|
| `srv/SetVelocity` | request: `linear`, `angular` — response: `success`, `message` |
| `srv/SetThreshold` | request: `threshold` — response: `success`, `message` |
| `srv/GetAverageVelocity` | response: `average_linear`, `average_angular` |
| `msg/ObstacleInfo` | `distance`, `direction`, `threshold` |
