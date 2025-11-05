# car_odometry

ROS2 package for odometry calculation and velocity command filtering for car-like robots.

## Overview

This package provides two nodes:

- **odometry_node**: Calculates odometry from motor RPM and IMU data, publishes odometry messages and TF transforms
- **cmd_vel_filter**: Filters and republishes velocity commands at a constant rate

## Installation

### Dependencies

```bash
sudo apt install ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-ament-cmake-python
```

### Build

```bash
# Build message dependencies first
colcon build --packages-select car_state_msg car_config_msg
source install/setup.bash

# Build this package
colcon build --packages-select car_odometry
source install/setup.bash
```

## Usage

### Launch both nodes

```bash
ros2 launch car_odometry car_system.launch.py
```

### Launch individual nodes

```bash
# Odometry node only
ros2 launch car_odometry car_odometry.launch.py

# Cmd vel filter only
ros2 launch car_odometry cmd_vel_filter.launch.py
```

### Run nodes directly

```bash
ros2 run car_odometry odometry_node.py
ros2 run car_odometry cmd_vel_filter.py
```

## Topics

### Subscribed

- `/car/car_state` (car_state_msg/CarState): Motor RPM and IMU data
- `/cmd_vel` (geometry_msgs/Twist): Input velocity commands (cmd_vel_filter only)

### Published

- `/car/odom` (nav_msgs/Odometry): Calculated odometry
- `/car/odom_twist` (geometry_msgs/Twist): Linear and angular velocities
- `/car/config` (car_config_msg/CarConfig): Car configuration (1 Hz)
- `/cmd_vel_filtered` (geometry_msgs/Twist): Filtered velocity commands at 20 Hz
- TF: `odom` → `base_link`

## Parameters

### odometry_node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wheel_radius` | double | 0.0325 | Wheel radius (meters) |
| `wheelbase` | double | 0.185 | Distance between front and rear axles (meters) |
| `track_width` | double | 0.15 | Distance between left and right wheels (meters) |
| `encoder_offset` | double | 187.5 | Steering encoder offset (degrees) |
| `max_steering_angle` | double | 30.0 | Maximum steering angle (degrees) |
| `max_rpm` | double | 300.0 | Maximum motor RPM |
| `velocity_threshold` | double | 0.001 | Minimum linear velocity (m/s) |
| `angular_threshold` | double | 0.001 | Minimum angular velocity (rad/s) |
| `publish_rate` | double | 50.0 | Odometry publishing rate (Hz) |
| `frame_id` | string | "odom" | Odometry frame ID |
| `child_frame_id` | string | "base_link" | Base link frame ID |

### cmd_vel_filter

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_rate_hz` | double | 20.0 | Publishing rate (Hz) |

## License

Apache 2.0
