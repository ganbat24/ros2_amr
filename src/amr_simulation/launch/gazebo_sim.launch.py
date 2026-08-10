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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_simulation_pkg = FindPackageShare('amr_simulation').find(
        'amr_simulation'
    )
    amr_control_pkg = FindPackageShare('amr_control').find('amr_control')

    default_world_path = os.path.join(
        amr_simulation_pkg, 'worlds', 'empty_world.sdf'
    )
    default_gui_config = os.path.join(
        amr_simulation_pkg, 'gazebo', 'gui_no_quickstart.config'
    )
    controller_config = os.path.join(
        amr_control_pkg, 'config', 'diff_drive_controller.yaml'
    )

    # Compose gz_args from launch arguments so that values forwarded by
    # an including launch file (e.g. amr_bringup system.launch.py) take
    # effect. Gazebo starts paused unless `-r` is given, so the `-r` flag
    # is only included when `paused` is "false".
    gz_args_value = PythonExpression(
        [
            '("-s " if "', LaunchConfiguration('headless'),
            '" == "true" else "")',
            '+ ("-r " if "', LaunchConfiguration('paused'),
            '" == "false" else "")',
            '+ "-v " + "', LaunchConfiguration('verbose'), '"',
            '+ " --gui-config " + "', LaunchConfiguration('gui_config'), '"',
            '+ " " + "', LaunchConfiguration('world'), '"',
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': gz_args_value,
        }.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            '/robot_description',
            '-entity',
            'amr',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.073',
        ],
        output='screen',
    )

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # gz /clock -> /clock_raw (ROS side remapped); clock_slow republishes
        # /clock at 20 Hz — the raw 100 Hz reliable stream backlogs every
        # node's clock subscription on slow hosts and their sim clocks drift.
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[('/clock', '/clock_raw')],
        output='screen',
    )

    clock_slow_node = Node(
        package='amr_sensors',
        executable='clock_slow.py',
        name='clock_slow',
        output='log',
        parameters=[{'use_sim_time': True}],
    )

    # Controller spawners — loaded after the robot is spawned. The
    # gz_ros2_control plugin creates the controller_manager; these
    # spawners load and activate the controllers into it.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            controller_config,
            # ros2_control 4.x / ros2_controllers 4.x: controllers publish
            # their topics under a namespace derived from the controller
            # name, and cmd_vel is TwistStamped. Restore the root-level
            # /odom + /cmd_vel contract the stack is built around.
            '--controller-ros-args',
            '-r /diff_drive_controller/odom:=/odom -r /diff_drive_controller/cmd_vel:=/cmd_vel',
        ],
    )

    return LaunchDescription(
        [
            # Fix Gazebo transport on WSL2 (multicast doesn't work)
            SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
            DeclareLaunchArgument(
                name='world',
                default_value=default_world_path,
                description='Absolute path to Gazebo world SDF file',
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
                description='Run Gazebo server-only (-s), no GUI',
            ),
            DeclareLaunchArgument(
                name='gui_config',
                default_value=default_gui_config,
                description='Gazebo GUI config file (no quick start)',
            ),
            gazebo,
            spawn_entity,
            bridge_clock,
            clock_slow_node,
            # Load joint_state_broadcaster after spawn completes
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            # Load diff_drive_controller after joint_state_broadcaster
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[diff_drive_controller_spawner],
                )
            ),
        ]
    )
