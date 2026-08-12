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
Localization: EKF + (SLAM Toolbox XOR AMCL + map_server).

SLAM and AMCL both publish map -> odom, so they are mutually exclusive
and selected via the `use_slam` launch argument (default: AMCL).
map_server, AMCL and slam_toolbox are lifecycle nodes; each mode gets
its own lifecycle_manager with autostart.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_localization_pkg = FindPackageShare('amr_localization').find(
        'amr_localization'
    )
    amr_navigation_pkg = FindPackageShare('amr_navigation').find(
        'amr_navigation'
    )

    ekf_config = os.path.join(amr_localization_pkg, 'config', 'ekf.yaml')
    slam_toolbox_config = os.path.join(
        amr_localization_pkg,
        'config',
        'slam_toolbox',
        'slam_toolbox_params.yaml',
    )
    amcl_config = os.path.join(
        amr_localization_pkg, 'config', 'amcl', 'amcl_params.yaml'
    )
    default_map_file = os.path.join(
        amr_navigation_pkg, 'maps', 'empty_map.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam')
    map_file = LaunchConfiguration('map')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    # Re-stamp the EKF's odom->base_link TF at /clock time: the diff_drive
    # controller's clock (inside gz_ros_control) lags /clock by seconds and
    # drifts, which otherwise makes AMCL/Nav2 drop everything against the
    # TF cache ("earlier than all the data").
    tf_restamper_node = Node(
        package='amr_localization',
        executable='tf_restamp.py',
        name='tf_restamper',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_toolbox_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_slam),
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {
                'use_sim_time': use_sim_time,
                # Override the YAML initial pose from launch args so the
                # stack is world-agnostic (defaults match the amr_office
                # spawn in amr_simulation/tools/generate_office_world.py).
                'initial_pose': {
                    'x': initial_x,
                    'y': initial_y,
                    'z': 0.0,
                    'yaw': initial_yaw,
                },
            },
        ],
        condition=UnlessCondition(use_slam),
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'yaml_filename': map_file}
        ],
        condition=UnlessCondition(use_slam),
    )

    # map_server and amcl are lifecycle nodes: configure + activate them
    # only in AMCL mode (in SLAM mode they are not launched).
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }
        ],
        condition=UnlessCondition(use_slam),
    )

    # slam_toolbox 2.8.x's async node is a LIFECYCLE node: without
    # configure+activate it subscribes to nothing and publishes no map
    # (it sat unconfigured with zero output — misdiagnosed earlier as an
    # upstream params regression). Drive it with a lifecycle manager in
    # SLAM mode, mirroring the AMCL pattern.
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }
        ],
        condition=IfCondition(use_slam),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='true',
                description='Use simulation clock',
            ),
            DeclareLaunchArgument(
                name='use_slam',
                default_value='false',
                description='Use SLAM Toolbox for mapping (true) or '
                'AMCL for localization with pre-built map (false). '
                'Default AMCL; SLAM mode drives slam_toolbox through its '
                'own lifecycle manager.',
            ),
            DeclareLaunchArgument(
                name='map',
                default_value=default_map_file,
                description='Map YAML file for AMCL/map_server mode',
            ),
            DeclareLaunchArgument(
                name='initial_x',
                default_value='1.5',
                description='AMCL initial pose x (map frame; must match '
                'the robot spawn of the active world)',
            ),
            DeclareLaunchArgument(
                name='initial_y',
                default_value='1.5',
                description='AMCL initial pose y (map frame)',
            ),
            DeclareLaunchArgument(
                name='initial_yaw',
                default_value='0.0',
                description='AMCL initial pose yaw (radians)',
            ),
            ekf_node,
            tf_restamper_node,
            slam_toolbox_node,
            amcl_node,
            map_server_node,
            lifecycle_manager,
            lifecycle_manager_slam,
        ]
    )
