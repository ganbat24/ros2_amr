# amr_sensors — Interface Spec

## Owned Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | Publish | IMU data (orientation, angular velocity, linear acceleration) |
| `/scan` | `sensor_msgs/LaserScan` | Publish | 2D LiDAR scan (360 samples, 0.1–8.0 m range, 10 Hz) |
| `/camera/image_raw` | `sensor_msgs/Image` | Publish | RGB raw image (320x240, 30 Hz) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Publish | Camera calibration info |
| `/image_proc/image_rect` | `sensor_msgs/Image` | Publish | Rectified image from image_proc |

## Owned Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_sim_time` | bool | `true` | Use simulation clock |

## Owned Frames

None owned.

## Owned Actions

None.

## Owned Services

None.
