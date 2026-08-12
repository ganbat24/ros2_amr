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
    nav2_bringup_pkg = FindPackageShare('nav2_bringup').find('nav2_bringup')

    default_nav_params = os.path.join(
        amr_navigation_pkg, 'config', 'nav2_params.yaml'
    )
    default_bt_xml_path = os.path.join(
        nav2_bringup_pkg,
        'behavior_trees',
        'navigate_w_replanning_and_recovery.xml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    paused = LaunchConfiguration('paused')
    verbose = LaunchConfiguration('verbose')
    headless = LaunchConfiguration('headless')
    gz_ip = LaunchConfiguration('gz_ip')
    use_slam = LaunchConfiguration('use_slam')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    bt_xml_filename = LaunchConfiguration('bt_xml_filename')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')

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
            'gz_ip': gz_ip,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_yaw': spawn_yaw,
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
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_yaw': initial_yaw,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_navigation_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'bt_xml_filename': bt_xml_filename,
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
                    amr_simulation_pkg, 'worlds', 'amr_office.sdf'
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
                name='gz_ip',
                default_value='127.0.0.1',
                description='Gazebo transport IP: 127.0.0.1 (loopback, '
                'for hosts without multicast) or a routable interface IP',
            ),
            DeclareLaunchArgument(
                name='use_slam',
                default_value='false',
                description='Use SLAM Toolbox for mapping (true) or AMCL '
                '(false). Default AMCL: slam_toolbox 2.8.5 has an upstream '
                'params regression; flip to true once upstream fixes it.',
            ),
            DeclareLaunchArgument(
                name='map',
                default_value=os.path.join(
                    amr_navigation_pkg, 'maps', 'amr_office.yaml'
                ),
                description='Map YAML for AMCL/map_server mode',
            ),
            DeclareLaunchArgument(
                name='spawn_x',
                default_value='1.5',
                description='Robot spawn x (world frame)',
            ),
            DeclareLaunchArgument(
                name='spawn_y',
                default_value='1.5',
                description='Robot spawn y (world frame)',
            ),
            DeclareLaunchArgument(
                name='spawn_yaw',
                default_value='0.0',
                description='Robot spawn yaw (radians)',
            ),
            DeclareLaunchArgument(
                name='initial_x',
                default_value='1.5',
                description='AMCL initial pose x (map frame; must match '
                'the robot spawn)',
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
            DeclareLaunchArgument(
                name='params_file',
                default_value=default_nav_params,
                description='Nav2 parameter YAML file',
            ),
            DeclareLaunchArgument(
                name='bt_xml_filename',
                default_value=default_bt_xml_path,
                description='Behavior tree XML file',
            ),
            rsp_launch,
            gazebo_sim_launch,
            sensors_launch,
            localization_launch,
            navigation_launch,
        ]
    )
