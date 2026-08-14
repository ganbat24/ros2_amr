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
from launch.conditions import IfCondition, UnlessCondition
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
            DeclareLaunchArgument(
                name='use_scan_restamp',
                default_value='false',
                description='Re-stamp scans at the latest map->odom TF time. '
                'Default false: measured on WSL2, it corrupts stamps rather '
                'than repairing them (see the note below). Enable only on a '
                'host where the raw bridge timestamps are provably stale.',
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            # Scan bridge. Two mutually exclusive forms, because disabling
            # the restamper must not leave /scan unpublished:
            #   restamper ON  -> bridge publishes /scan_raw, restamper -> /scan
            #   restamper OFF -> bridge publishes /scan directly
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                remappings=[('/scan', '/scan_raw')],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('use_scan_restamp')),
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=UnlessCondition(
                    LaunchConfiguration('use_scan_restamp')),
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
            # The restamper stamps each scan at the latest map->odom TF
            # time. It was added because raw gpu_lidar timestamps were
            # observed lagging the physics clock by seconds on a 2-core host.
            #
            # Measured on WSL2 2026-08-14 with `amr_metrics scan_health`, in
            # one run, both topics side by side:
            #     /scan_raw (bridge)     0 non-increasing stamps, age +0.020 s
            #     /scan     (restamped)  128 of 194 non-increasing, age -0.100 s
            #
            # So on this host it is not repairing stamps, it is destroying
            # them. map->odom is published by AMCL and only advances when AMCL
            # updates, so every scan arriving between two AMCL updates gets
            # the same timestamp — and tf2 message filters discard repeated
            # stamps. The shim added to stop AMCL dropping scans was making
            # AMCL drop scans.
            #
            # Default off. Re-enable only after measuring that the raw
            # timestamps are actually stale on the host in question.
            Node(
                package='amr_sensors',
                executable='scan_restamp.py',
                name='scan_restamper',
                output='log',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('use_scan_restamp')),
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
                # /tf publish that late subscribers miss entirely). Parent is
                # base_link and the gz-scoped sensor frame is the CHILD: an
                # edge pointing INTO base_link would give base_link a second
                # parent and fork the TF tree ("two or more unconnected
                # trees"), breaking every consumer's chain lookup.
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'amr/base_footprint/laser',
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
