# amr_sensors — Dependencies

## Upstream (Consumed)

| Topic | Type | Consumed By |
|-------|------|-------------|
| `/imu/data` (gz transport) | `ignition.msgs.IMU` | `ros_gz_bridge` |
| `/scan` (gz transport) | `ignition.msgs.LaserScan` | `ros_gz_bridge` |
| `/camera/image_raw` (gz transport) | `ignition.msgs.Image` | `ros_gz_bridge` |
| `/camera/camera_info` (gz transport) | `ignition.msgs.CameraInfo` | `ros_gz_bridge` |

## Downstream (Produced)

| Topic | Type | Consumed By |
|-------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | `robot_localization` EKF |
| `/scan` | `sensor_msgs/LaserScan` | `slam_toolbox`, `amcl`, Nav2 costmap |
| `/camera/image_raw` | `sensor_msgs/Image` | `image_proc` |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | `image_proc` |

## External Dependencies

- `ros_gz_bridge`
- `ros_gz_image`
- `image_proc`
