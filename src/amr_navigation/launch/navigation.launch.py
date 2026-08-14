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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    amr_navigation_pkg = FindPackageShare('amr_navigation').find(
        'amr_navigation'
    )
    # bt_navigator's plugin-based navigators each declare their own BT
    # XML param (default_nav_to_pose_bt_xml / default_nav_through_poses_
    # bt_xml) — nav2_bringup no longer ships a behavior_trees/ dir at
    # all in this nav2 release, and there is no generic
    # default_bt_xml_filename param anymore either. The default tree
    # lives in nav2_bt_navigator's own share dir.
    nav2_bt_navigator_pkg = FindPackageShare('nav2_bt_navigator').find(
        'nav2_bt_navigator'
    )

    default_nav_params = os.path.join(
        amr_navigation_pkg, 'config', 'nav2_params.yaml'
    )
    default_bt_xml_path = os.path.join(
        nav2_bt_navigator_pkg,
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    bt_xml_filename = LaunchConfiguration('bt_xml_filename')

    # Rewrite the Nav2 YAML at launch time so that use_sim_time and the
    # behavior tree are taken from launch arguments.
    nav2_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': bt_xml_filename,
        },
        convert_types=True,
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
    )

    # nav2_velocity_smoother outputs plain Twist on /cmd_vel_smoothed but
    # ros2_controllers 4.x diff_drive subscribes TwistStamped (on
    # /cmd_vel_stamped — see the spawner remap in gazebo_sim.launch.py).
    twist_to_stamped = Node(
        package='amr_navigation',
        executable='twist_to_stamped.py',
        name='twist_to_stamped',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # The lifecycle manager must not create its service clients during the
    # 40-node discovery storm: clients created then can block forever waiting
    # to discover the managed nodes' lifecycle services (the planner_server
    # activation call times out after ~2 min and the manager aborts bringup,
    # leaving every nav node inactive). nav2_lifecycle_manager declares no
    # parameter that bounds that initial wait — verified against
    # libnav2_lifecycle_manager_core.so, which declares only autostart,
    # node_names, bond_timeout, bond_respawn_max_duration and
    # attempt_respawn_reconnection — so the wait cannot be made to fail fast.
    #
    # This used to be a flat TimerAction(period=60.0) counted from launch
    # time, which assumed a fixed machine speed. Instead, start counting from
    # the moment the last managed node's process actually starts, so the delay
    # tracks the host: fast hosts activate seconds after the nodes are up,
    # slow hosts get proportionally more room without anyone editing a
    # constant. `lifecycle_settle` remains tunable for pathologically slow
    # environments.
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[nav2_params],
    )

    lifecycle_manager = RegisterEventHandler(
        OnProcessStart(
            target_action=bt_navigator,
            on_start=[
                TimerAction(
                    period=LaunchConfiguration('lifecycle_settle'),
                    actions=[lifecycle_manager_node],
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='params_file',
                default_value=default_nav_params,
                description='Nav2 parameter YAML file',
            ),
            DeclareLaunchArgument(
                name='use_sim_time',
                default_value='true',
                description='Use simulation clock',
            ),
            DeclareLaunchArgument(
                name='bt_xml_filename',
                default_value=default_bt_xml_path,
                description='Behavior tree XML file',
            ),
            DeclareLaunchArgument(
                name='lifecycle_settle',
                default_value='8.0',
                description='Seconds to wait after the last managed node '
                'starts before the lifecycle manager creates its service '
                'clients. Counted from node start, not from launch, so it '
                'tracks host speed. Raise it on a pathologically slow host.',
            ),
            controller_server,
            planner_server,
            smoother_server,
            behavior_server,
            bt_navigator,
            velocity_smoother,
            twist_to_stamped,
            lifecycle_manager,
        ]
    )
