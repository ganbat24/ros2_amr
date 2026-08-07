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
"""Step 4 — Localization (EKF + SLAM Toolbox or AMCL).

Adds robot_localization EKF for sensor fusion, and either:
  - SLAM Toolbox for online mapping (use_slam=true, default)
  - AMCL + map_server for localization with a pre-built map (use_slam=false)

SLAM and AMCL are mutually exclusive — both publish map → odom.

Requires: Steps 1–3 (description + Gazebo + sensors).

Verify:
  - ros2 topic echo /tf --once           (odom → base_link transform)
  - ros2 node list | grep ekf
  - ros2 node list | grep slam (or amcl)
  - Drive the robot; SLAM should build a map in RViz
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Include Steps 1–3 (description + Gazebo + sensors)
    step_03_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'step_03_sensors.launch.py',
            )
        ),
    )

    # Extended Kalman Filter — fuses odom + IMU
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    # SLAM Toolbox — online mapping (when use_slam=true)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_toolbox_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_slam),
    )

    # AMCL — global localization (when use_slam=false)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_slam),
    )

    # Map server — serves a pre-built map for AMCL (when use_slam=false)
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
            step_03_launch,
            ekf_node,
            slam_toolbox_node,
            amcl_node,
            map_server_node,
            lifecycle_manager,
        ]
    )
