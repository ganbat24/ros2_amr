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
"""
Step 3 — Sensor Bridges (LiDAR, IMU, Camera).

Bridges Gazebo sensor topics to ROS 2:
  /scan           (LiDAR)
  /imu/data       (IMU)
  /camera/image_raw  (RGB camera)
  /camera/camera_info
  + image_proc rectify node

Requires: Steps 1–2 (description + Gazebo with controllers).

Verify:
  - ros2 topic echo /scan --once         (laser scan data)
  - ros2 topic echo /imu/data --once     (IMU quaternion + angular vel)
  - ros2 topic echo /camera/image_raw --once  (camera image)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Include Steps 1–2 (description + Gazebo with controllers)
    step_02_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'step_02_gazebo.launch.py',
            )
        ),
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Bridge IMU: Gazebo → ROS 2
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Bridge LiDAR: Gazebo → ROS 2
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # gz topic /scan stays /scan on the gz side; the ROS side is
        # remapped to /scan_raw for the receive-time restamper.
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        remappings=[('/scan', '/scan_raw')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Bridge Camera image: Gazebo → ROS 2
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Bridge Camera info: Gazebo → ROS 2
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Image rectification
    image_proc_node = Node(
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
    )

    # Re-stamp scans at receive time: under headless software rendering
    # the gpu_lidar timestamps lag the physics clock by seconds, which
    # makes AMCL/SLAM/costmaps drop every scan against the TF cache.
    scan_restamper = Node(
        package='amr_sensors',
        executable='scan_restamp.py',
        name='scan_restamper',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # The gz URDF importer attaches all sensors to the model root, so
    # scan/camera messages carry gz-scoped frames not in the ROS TF tree.
    # Identity static TFs bridge them to the intended ROS frames (x/y
    # offsets are zero; z/rotation only affect 3D consumers).
    sensor_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # new-style args: latched /tf_static (old-style is a one-shot
        # /tf publish that late subscribers miss entirely). Parent is
        # base_link and the gz-scoped sensor frame is the CHILD: an
        # edge pointing INTO base_link would give base_link a second
        # parent and fork the TF tree ("two or more unconnected trees"),
        # breaking every consumer's chain lookup.
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'amr/base_footprint/laser',
        ],
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    camera_frame_tf = Node(
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
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='true',
                description='Use simulation clock',
            ),
            step_02_launch,
            bridge_imu,
            bridge_lidar,
            bridge_camera,
            bridge_camera_info,
            image_proc_node,
            sensor_frame_tf,
            camera_frame_tf,
            scan_restamper,
        ]
    )
