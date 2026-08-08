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
Standalone controller_manager bringup (real hardware).

Starts robot_state_publisher + a standalone ros2_control_node and
loads joint_state_broadcaster and diff_drive_controller via spawners.

For SIMULATION use amr_simulation/gazebo_sim.launch.py instead: the
gz_ros2_control plugin creates its own controller_manager inside
Gazebo, so a standalone ros2_control_node is not needed (and would
conflict) there.

For real hardware, the <ros2_control> block in the URDF must point at
the actual hardware interface plugin (e.g. a CAN/GPIO adapter) instead
of gz_ros2_control/GazeboSimSystem.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr_control_pkg = FindPackageShare('amr_control').find('amr_control')
    amr_description_pkg = FindPackageShare('amr_description').find(
        'amr_description'
    )
    controller_manager_config = os.path.join(
        amr_control_pkg, 'config', 'controller_manager.yaml'
    )
    diff_drive_config = os.path.join(
        amr_control_pkg, 'config', 'diff_drive_controller.yaml'
    )

    default_model_path = os.path.join(
        amr_description_pkg, 'urdf', 'amr.urdf.xacro'
    )

    robot_description = {
        'robot_description': Command(
            ['xacro ', LaunchConfiguration('robot_description')]
        )
    }

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_description_pkg, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'robot_description': LaunchConfiguration('robot_description')
        }.items(),
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            LaunchConfiguration('config_file'),
        ],
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel')],
    )

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
            diff_drive_config,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='config_file',
                default_value=controller_manager_config,
                description='Absolute path to controller_manager YAML config',
            ),
            DeclareLaunchArgument(
                name='robot_description',
                default_value=default_model_path,
                description='Absolute path to the robot '
                'description xacro file',
            ),
            rsp_launch,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]
    )
