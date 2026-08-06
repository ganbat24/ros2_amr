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
"""Localization: EKF + (SLAM Toolbox XOR AMCL + map_server).

SLAM and AMCL both publish map -> odom, so they are mutually exclusive
and selected via the `use_slam` launch argument (default: SLAM).
map_server and AMCL are lifecycle nodes and are driven by their own
lifecycle_manager in AMCL mode.
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

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
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
        parameters=[amcl_config, {'use_sim_time': use_sim_time}],
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='true',
                description='Use simulation clock',
            ),
            DeclareLaunchArgument(
                name='use_slam',
                default_value='true',
                description='Use SLAM Toolbox for mapping (true) or '
                'AMCL for localization with pre-built map (false)',
            ),
            DeclareLaunchArgument(
                name='map',
                default_value=default_map_file,
                description='Map YAML file for AMCL/map_server mode',
            ),
            ekf_node,
            slam_toolbox_node,
            amcl_node,
            map_server_node,
            lifecycle_manager,
        ]
    )
