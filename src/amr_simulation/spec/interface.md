# amr_simulation — Interface Spec

## Owned Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/clock` | `rosgraph_msgs/Clock` | Publish | Simulation clock (bridged by `ros_gz_bridge`) |

## Description

This package launches Gazebo Harmonic and spawns the robot model. Sensor data
(LiDAR, IMU, camera) is produced on Gazebo transport topics and bridged to
ROS 2 by `ros_gz_bridge` in `amr_sensors`. The `gz_ros2_control` plugin
creates its own `controller_manager` and loads controllers defined in
`amr_control/config/controller_manager.yaml`.

## Owned Parameters

None owned directly by this package.

## Owned Frames

None. TF is published by `robot_state_publisher` in `amr_description`.

## Owned Actions

None.

## Owned Services

None.
