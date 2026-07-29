# amr_simulation — Interface Spec

## Owned Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | Publish | IMU data from Gazebo Harmonic |
| `/scan` | `sensor_msgs/LaserScan` | Publish | 2D LiDAR scan data from Gazebo Harmonic |
| `/camera/image_raw` | `sensor_msgs/Image` | Publish | RGB camera raw image from Gazebo Harmonic |

## Owned Parameters

None owned directly by this package. Bridge nodes accept command-line args.

## Owned Frames

None. TF is published by `robot_state_publisher` in `amr_description`.

## Owned Actions

None.

## Owned Services

None.
