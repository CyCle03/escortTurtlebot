#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
#
# Authors: Hyungyu Kim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    number_of_follower = int(LaunchConfiguration('number_of_follower').perform(context))
    use_lidar_bridge = LaunchConfiguration('use_lidar_bridge').perform(context).lower() in ('1', 'true', 'yes')
    use_map_tracking = LaunchConfiguration('use_map_tracking').perform(context).lower() in ('1', 'true', 'yes')
    tracking_frame = LaunchConfiguration('tracking_frame').perform(context)
    initial_dx = LaunchConfiguration('initial_dx').perform(context)
    initial_dy = LaunchConfiguration('initial_dy').perform(context)

    if number_of_follower < 1 or number_of_follower > 4:
        raise RuntimeError('number_of_follower must be between 1 and 4')

    follower = Node(
        package='escort_follower',
        executable='follower',
        output='screen',
        arguments=[str(number_of_follower)],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'follow_distance': 0.5},
            {'publish_odom_bridge': False},
            {'tracking_frame': tracking_frame},
        ]
    )
    nodes = []
    nodes.append(follower)
    for i in range(number_of_follower):
        namespace = f'TB3_{i+2}'
        leader_namespace = f'TB3_{i+1}'
        tf_bridge_node = None
        if not use_map_tracking:
            if use_lidar_bridge:
                tf_bridge_node = Node(
                    package='escort_turtlebot_pkg',
                    executable='lidar_odom_bridge',
                    name=f'lidar_odom_bridge_{leader_namespace}_to_{namespace}',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'scan_topic': f'/{namespace}/scan'},
                        {'leader_ns': leader_namespace},
                        {'follower_ns': namespace},
                        {'target_bearing_deg': 0.0},
                        {'search_half_angle_deg': 35.0},
                        {'min_range': 0.12},
                        {'max_range': 3.0},
                        {'smoothing_alpha': 0.35},
                        {'min_candidate_points': 3},
                        {'max_tracking_jump': 0.6},
                    ],
                )
            else:
                tf_bridge_node = Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name=f'odom_bridge_{leader_namespace}_to_{namespace}',
                    output='screen',
                    arguments=[
                        str(initial_dx), str(initial_dy), '0',
                        '0', '0', '0',
                        f'{leader_namespace}/odom',
                        f'{namespace}/odom',
                    ],
                )

        custom_ctrl_yaml_path = os.path.join(
            get_package_share_directory('escort_turtlebot_pkg'),
            'param',
            f'escort_controll_server{i+1}.yaml'
        )
        default_ctrl_yaml_path = os.path.join(
            get_package_share_directory('escort_follower'),
            'param',
            f'controll_server{i+1}.yaml'
        )
        ctrl_yaml_path = custom_ctrl_yaml_path if os.path.exists(custom_ctrl_yaml_path) else default_ctrl_yaml_path

        ctrl_node = Node(
            package='nav2_controller',
            executable='controller_server',
            namespace=namespace,
            name='controller_server',
            output='screen',
            parameters=[
                ctrl_yaml_path,
                {'use_sim_time': use_sim_time}],
            remappings=[
                (f'/{namespace}/cmd_vel', f'/{namespace}/cmd_vel_not_smoothed')]
        )

        lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=namespace,
            name='lifecycle_manager_controller',
            output='screen',
            parameters=[
                {'autostart': True},
                {'use_sim_time': use_sim_time},
                {'node_names': ['controller_server', 'velocity_smoother']}],
        )

        velocity_smoother_node = Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            namespace=namespace,
            name='velocity_smoother',
            output='screen',
            parameters=[ctrl_yaml_path],
            remappings=[
                (f'/{namespace}/cmd_vel', f'/{namespace}/cmd_vel_not_smoothed'),
                (f'/{namespace}/cmd_vel_smoothed', f'/{namespace}/cmd_vel')]
        )
        if tf_bridge_node is not None:
            nodes.append(tf_bridge_node)
        nodes.append(
            TimerAction(
                period=3.0,
                actions=[ctrl_node, lifecycle_node, velocity_smoother_node],
            )
        )

    return nodes


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'number_of_follower',
            default_value='1',
            description='Number of follower robots'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'use_lidar_bridge',
            default_value='true',
            description='Use LiDAR-based odom bridge instead of static transform'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'use_map_tracking',
            default_value='true',
            description='Track in shared frame (map) and skip odom bridging'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'tracking_frame',
            default_value='map',
            description='Shared frame used by follower path generation'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'initial_dx',
            default_value='-0.6',
            description='Static bridge x (leader odom -> follower odom) when LiDAR bridge is disabled'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'initial_dy',
            default_value='0.0',
            description='Static bridge y (leader odom -> follower odom) when LiDAR bridge is disabled'
        )
    )
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
