# amr_simulation — Dependencies

## Upstream (Consumed)

| Topic | Type | Consumed By |
|-------|------|-------------|
| `/robot_description` | `std_msgs/String` | `ros_gz_sim create` |

## Downstream (Produced)

| Topic | Type | Consumed By |
|-------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | `robot_localization` EKF |
| `/scan` | `sensor_msgs/LaserScan` | `slam_toolbox`, `amcl`, Nav2 costmap |
| `/camera/image_raw` | `sensor_msgs/Image` | `image_proc` |

## External Dependencies

- `ros_gz_sim`
- `gz_ros2_control`
- `ros_gz_bridge`
- `ros_gz_image`
- `xacro`
