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
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_gz_args(context):
    """
    Compose the gz sim CLI args from launch configurations.

    Each value is shell-quoted individually (PythonExpression string
    concatenation is quote-injection-prone: a world path containing
    spaces or quotes would corrupt the argument list).
    """
    headless = context.launch_configurations.get('headless', 'false')
    paused = context.launch_configurations.get('paused', 'false')
    verbose = context.launch_configurations.get('verbose', '4')
    gui_config = context.launch_configurations.get('gui_config')
    world = context.launch_configurations.get('world')

    # Gazebo starts paused unless `-r` is given, so `-r` is only added
    # when the `paused` argument is "false".
    args = []
    if headless == 'true':
        args.append('-s')
    if paused == 'false':
        args.append('-r')
    args.extend(['-v', verbose])
    args.extend(['--gui-config', gui_config])
    args.append(world)
    return [
        SetLaunchConfiguration(
            'gz_args', ' '.join(shlex.quote(a) for a in args)
        )
    ]


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

    # gz_args is computed at launch time by _build_gz_args (above) from
    # the launch configurations, so values forwarded by an including
    # launch file (e.g. amr_bringup system.launch.py) take effect.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': LaunchConfiguration('gz_args'),
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
        # `[` = GZ->ROS direction (the `]` marker is ROS->GZ, NOT a
        # reliability flag). Consumers subscribe /clock best-effort, so a
        # reliable publisher is compatible and cheap to drop on slow hosts;
        # the historical "clock backlog" was a symptom of the tf_restamp
        # self-loop CPU spin (fixed), not of the reliable clock itself.
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
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
            # /odom contract and point the controller at the relay's
            # /cmd_vel_stamped (see amr_navigation/twist_to_stamped.py) so
            # /cmd_vel stays a pure Twist bus for the Nav2/teleop chain.
            '--controller-ros-args',
            '-r /diff_drive_controller/odom:=/odom '
            '-r /diff_drive_controller/cmd_vel:=/cmd_vel_stamped',
        ],
    )

    return LaunchDescription(
        [
            # gz_ip must be declared before SetEnvironmentVariable reads it
            # (standalone launches don't forward it from an including file).
            DeclareLaunchArgument(
                name='gz_ip',
                default_value='127.0.0.1',
                description='Gazebo transport IP: 127.0.0.1 (loopback, '
                'for hosts without multicast) or a routable interface IP',
            ),
            # Gazebo transport over loopback (works on hosts without
            # multicast, e.g. WSL2); override gz_ip for real interfaces.
            SetEnvironmentVariable(
                'GZ_IP', LaunchConfiguration('gz_ip')
            ),
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
            OpaqueFunction(function=_build_gz_args),
            gazebo,
            spawn_entity,
            bridge_clock,
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
