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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_pkg = FindPackageShare('amr_description').find(
        'amr_description'
    )
    amr_simulation_pkg = FindPackageShare('amr_simulation').find(
        'amr_simulation'
    )

    default_model_path = os.path.join(
        robot_description_pkg, 'urdf', 'amr.urdf.xacro'
    )
    default_world_path = os.path.join(
        amr_simulation_pkg, 'worlds', 'empty_world.sdf'
    )
    default_gui_config = os.path.join(
        amr_simulation_pkg, 'gazebo', 'gui_no_quickstart.config'
    )

    # Build gz_args with resolved paths (available at parse time)
    gz_args_value = (
        f'-r -v 4'
        f' --gui-config {default_gui_config}'
        f' {default_world_path}'
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
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='model',
                default_value=default_model_path,
                description='Absolute path to robot xacro file',
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
                name='gui_config',
                default_value=default_gui_config,
                description='Gazebo GUI config file (no quick start)',
            ),
            gazebo,
            spawn_entity,
            bridge_clock,
        ]
    )
