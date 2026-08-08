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
Top-level bringup: Gazebo + AMR with control, sensors, localization, nav.

The gz_ros2_control plugin (GazeboSimROS2ControlPlugin) inside the
URDF creates its own controller_manager; gazebo_sim.launch.py spawns
the controllers into it after the robot appears.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_description_pkg = FindPackageShare('amr_description').find(
        'amr_description'
    )
    amr_simulation_pkg = FindPackageShare('amr_simulation').find(
        'amr_simulation'
    )
    amr_sensors_pkg = FindPackageShare('amr_sensors').find('amr_sensors')
    amr_localization_pkg = FindPackageShare('amr_localization').find(
        'amr_localization'
    )
    amr_navigation_pkg = FindPackageShare('amr_navigation').find(
        'amr_navigation'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    paused = LaunchConfiguration('paused')
    verbose = LaunchConfiguration('verbose')
    headless = LaunchConfiguration('headless')
    use_slam = LaunchConfiguration('use_slam')
    map_file = LaunchConfiguration('map')

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_description_pkg, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'robot_description': os.path.join(
                amr_description_pkg, 'urdf', 'amr.urdf.xacro'
            ),
            'use_joint_state_publisher': 'false',
        }.items(),
    )

    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_simulation_pkg, 'launch', 'gazebo_sim.launch.py')
        ),
        launch_arguments={
            'world': world,
            'paused': paused,
            'verbose': verbose,
            'headless': headless,
        }.items(),
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_sensors_pkg, 'launch', 'sensors.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                amr_localization_pkg, 'launch', 'localization.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_slam': use_slam,
            'map': map_file,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_navigation_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='true',
                description='Use simulation clock',
            ),
            DeclareLaunchArgument(
                name='world',
                default_value=os.path.join(
                    amr_simulation_pkg, 'worlds', 'empty_world.sdf'
                ),
                description='Gazebo world SDF',
            ),
            DeclareLaunchArgument(
                name='paused',
                default_value='false',
                description='Start Gazebo paused',
            ),
            DeclareLaunchArgument(
                name='verbose',
                default_value='4',
                description='Gazebo verbose output level (0-4)',
            ),
            DeclareLaunchArgument(
                name='headless',
                default_value='false',
                description='Run Gazebo server-only, no GUI',
            ),
            DeclareLaunchArgument(
                name='use_slam',
                default_value='true',
                description='Use SLAM Toolbox (true) or AMCL (false)',
            ),
            DeclareLaunchArgument(
                name='map',
                default_value=os.path.join(
                    amr_navigation_pkg, 'maps', 'empty_map.yaml'
                ),
                description='Map YAML for AMCL/map_server mode',
            ),
            rsp_launch,
            gazebo_sim_launch,
            sensors_launch,
            localization_launch,
            navigation_launch,
        ]
    )
