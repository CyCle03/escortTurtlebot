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
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    leader_name = LaunchConfiguration('leader_name')
    follower_name = LaunchConfiguration('follower_name')
    follow_distance = LaunchConfiguration('follow_distance')
    initial_step_distance = LaunchConfiguration('initial_step_distance')

    follower = Node(
        package='escort_follower',
        executable='follower',
        name=[follower_name, '_follower_node'],
        output='screen',
        arguments=[follower_name, leader_name],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'follow_distance': follow_distance},
            {'initial_step_distance': initial_step_distance},
            {'tracking_frame': [leader_name, '/odom']},
        ]
    )
    namespace = follower_name
    ctrl_yaml_path = os.path.join(
        get_package_share_directory('escort_follower'),
        'param',
        'controll_server1.yaml'
    )

    tf_bridge_node = Node(
        package='escort_turtlebot_pkg',
        executable='follower_detector_node',
        name='follower_detector_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'leader_name': leader_name},
            {'follower_name': follower_name}
        ]
    )

    ctrl_node = Node(
        package='nav2_controller',
        executable='controller_server',
        namespace=namespace,
        name='controller_server',
        output='screen',
        parameters=[ctrl_yaml_path, {'use_sim_time': use_sim_time}],
        remappings=[([namespace, '/cmd_vel'], [namespace, '/cmd_vel_not_smoothed'])]
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
            ([namespace, '/cmd_vel'], [namespace, '/cmd_vel_not_smoothed']),
            ([namespace, '/cmd_vel_smoothed'], [namespace, '/cmd_vel'])]
    )

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
            'leader_name',
            default_value='TB3_1',
            description='Name of the leader robot'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'follower_name',
            default_value='TB3_2',
            description='Name of the follower robot'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'follow_distance',
            default_value='0.5',
            description='Target center-to-center distance from leader (meters)'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'initial_step_distance',
            default_value='0.0',
            description='Initial one-time forward step distance toward target (meters)'
        )
    )
    ld.add_action(tf_bridge_node)
    ld.add_action(follower)
    ld.add_action(
        TimerAction(
            period=3.0,
            actions=[ctrl_node, lifecycle_node, velocity_smoother_node],
        )
    )
    return ld
