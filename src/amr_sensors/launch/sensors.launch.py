# Copyright 2026 Ganbat Selenge
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='true',
                description='Use simulation clock',
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                # gz topic /scan stays /scan on the gz side; the ROS side is
                # remapped to /scan_raw for the receive-time restamper.
                arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                remappings=[('/scan', '/scan_raw')],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/camera/camera_info'
                    '@sensor_msgs/msg/CameraInfo'
                    '[gz.msgs.CameraInfo'
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='image_proc',
                executable='rectify_node',
                name='image_proc',
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/image_proc/image_rect'),
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            # Re-stamp scans at receive time: under headless software
            # rendering the gpu_lidar timestamps lag the physics clock by
            # seconds, which makes AMCL/SLAM/costmaps drop every scan
            # against the TF cache. The relay aligns /scan with /odom.
            Node(
                package='amr_sensors',
                executable='scan_restamp.py',
                name='scan_restamper',
                output='log',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            # The gz URDF importer attaches all sensors to the model root,
            # so scan/camera messages carry gz-scoped frames that are not
            # in the ROS TF tree. Publish identity static TFs to the
            # intended ROS frames (x/y offsets are zero; z/rotation only
            # affect 3D consumers, not 2D SLAM or image rectification).
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                # new-style args: latched /tf_static (old-style is a one-shot
                # /tf publish that late subscribers miss entirely)
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'amr/base_footprint/laser',
                    '--child-frame-id', 'laser_frame',
                ],
                output='log',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'amr/base_footprint/camera',
                    '--child-frame-id', 'camera_optical_frame',
                ],
                output='log',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
    )
