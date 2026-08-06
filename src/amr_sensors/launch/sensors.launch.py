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
                arguments=['/imu/data@sensor_msgs/Imu[gz.msgs.IMU'],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/scan@sensor_msgs/LaserScan[gz.msgs.LaserScan'],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/camera/image_raw@sensor_msgs/Image[gz.msgs.Image'
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/camera/camera_info'
                    '@sensor_msgs/CameraInfo'
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
        ]
    )
