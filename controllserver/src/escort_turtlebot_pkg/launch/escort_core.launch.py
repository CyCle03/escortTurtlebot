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

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    declare_number_of_follower = DeclareLaunchArgument(
        'number_of_follower',
        default_value='1',
        description='Number of follower robots'
    )

    launch_dir = os.path.join(get_package_share_directory('escort_turtlebot_pkg'), 'launch')
    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'escort_follower.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'number_of_follower': number_of_follower,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_number_of_follower)
    ld.add_action(follower_launch)
    return ld
