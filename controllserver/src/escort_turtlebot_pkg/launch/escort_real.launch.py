#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    number_of_follower = LaunchConfiguration('number_of_follower')
    odom_bridge_x = LaunchConfiguration('odom_bridge_x')
    odom_bridge_y = LaunchConfiguration('odom_bridge_y')

    launch_dir = os.path.join(get_package_share_directory('escort_turtlebot_pkg'), 'launch')
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'escort_core.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'number_of_follower': number_of_follower,
            'odom_bridge_x': odom_bridge_x,
            'odom_bridge_y': odom_bridge_y,
        }.items(),
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'escort_slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'number_of_follower',
            default_value='1',
            description='Number of follower robots',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'odom_bridge_x',
            default_value='-0.50',
            description='Static TF x offset from TB3_1/odom to TB3_2/odom'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'odom_bridge_y',
            default_value='0.0',
            description='Static TF y offset from TB3_1/odom to TB3_2/odom'
        )
    )
    ld.add_action(core_launch)
    ld.add_action(slam_launch)

    sonar_process = ExecuteProcess(
        cmd=['ssh', 'penguin@192.168.0.201', 'bash', '-c', '"export ROS_DOMAIN_ID=116 && source /opt/ros/humble/setup.bash && python3 ~/sonar_pub.py"'],
        output='screen'
    )
    ld.add_action(sonar_process)

    return ld
