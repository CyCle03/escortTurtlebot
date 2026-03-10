#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    number_of_follower = LaunchConfiguration('number_of_follower')

    launch_dir = os.path.join(get_package_share_directory('escort_turtlebot_pkg'), 'launch')
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'escort_core.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'number_of_follower': number_of_follower,
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
    ld.add_action(core_launch)
    ld.add_action(slam_launch)
    return ld
