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
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


# (package, registered plugin, node name) for the composed nav2 stack.
#
# Every plugin string here was read off this machine's
# share/ament_index/resource_index/rclcpp_components/<package> index rather
# than guessed — note behavior_server::BehaviorServer, which does NOT follow
# the nav2_<pkg>::<Class> pattern the other six do. test_navigation_launch
# re-checks this list against that index, so a wrong name fails a unit test
# instead of a 15-minute sim run.
#
# Order matters: LoadComposableNodes loads sequentially and the lifecycle
# manager must come last, so it cannot create service clients before the
# servers it manages exist.
NAV2_COMPONENTS = [
    ('nav2_controller', 'nav2_controller::ControllerServer',
     'controller_server'),
    ('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server'),
    ('nav2_smoother', 'nav2_smoother::SmootherServer', 'smoother_server'),
    ('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server'),
    ('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator'),
    ('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother',
     'velocity_smoother'),
    ('nav2_waypoint_follower', 'nav2_waypoint_follower::WaypointFollower',
     'waypoint_follower'),
    ('nav2_lifecycle_manager', 'nav2_lifecycle_manager::LifecycleManager',
     'lifecycle_manager'),
]


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
    use_composition = LaunchConfiguration('use_composition')

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

    # Two ways to run the same six servers. `use_composition:=false` starts
    # each as its own process; `true` loads them all into one container.
    # Process-per-node remains the default until a composed campaign has been
    # measured against the process-per-node baseline — the capability landing
    # is not the same thing as the capability being better here.
    separate = UnlessCondition(use_composition)

    controller_server = Node(
        condition=separate,
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params],
    )

    planner_server = Node(
        condition=separate,
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    smoother_server = Node(
        condition=separate,
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params],
    )

    behavior_server = Node(
        condition=separate,
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        condition=separate,
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    velocity_smoother = Node(
        condition=separate,
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
    )

    waypoint_follower = Node(
        condition=separate,
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
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
        condition=separate,
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
        ),
        condition=separate,
    )

    # The composed path. Six servers plus the lifecycle manager in one
    # process, which removes the problem the settle timer above works around
    # rather than timing around it: those seven nodes become one DDS
    # participant instead of seven, so the discovery storm the lifecycle
    # manager could hang in no longer involves them at all.
    #
    # component_container_isolated, not component_container: each component
    # gets its own single-threaded executor on its own thread. A shared
    # single-threaded executor deadlocks here, because the lifecycle manager
    # calls change_state on servers that would be waiting in the same
    # executor for it to return.
    #
    # LoadComposableNodes loads in order, and the lifecycle manager is last,
    # so it cannot create its service clients before the servers it manages
    # exist. That is a stronger guarantee than the timer it replaces — an
    # ordering rather than a wait — which is why no settle delay appears on
    # this path.
    nav2_container = Node(
        condition=IfCondition(use_composition),
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[nav2_params],
        output='screen',
    )

    load_nav2_components = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container='/nav2_container',
        composable_node_descriptions=[
            ComposableNode(package=package, plugin=plugin, name=name,
                           parameters=[nav2_params])
            for package, plugin, name in NAV2_COMPONENTS
        ],
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
                'tracks host speed. Raise it on a pathologically slow host. '
                'Only applies when use_composition is false — the composed '
                'path orders the loads instead of waiting.',
            ),
            DeclareLaunchArgument(
                name='use_composition',
                default_value='false',
                description='Load the nav2 servers into a single '
                'component container instead of running each as its own '
                'process. Set false to isolate a crashing server, or to '
                'compare against the process-per-node baseline.',
            ),
            controller_server,
            planner_server,
            smoother_server,
            behavior_server,
            bt_navigator,
            velocity_smoother,
            waypoint_follower,
            twist_to_stamped,
            lifecycle_manager,
            nav2_container,
            load_nav2_components,
        ]
    )
