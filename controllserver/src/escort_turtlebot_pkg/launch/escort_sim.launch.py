#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def _launch_setup(context):
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    namespace_prefix = 'TB3'
    number_of_robots = 2

    use_sim_time = LaunchConfiguration('use_sim_time')
    leader_x = LaunchConfiguration('leader_x').perform(context)
    leader_y = LaunchConfiguration('leader_y').perform(context)
    follower_x = LaunchConfiguration('follower_x').perform(context)
    follower_y = LaunchConfiguration('follower_y').perform(context)
    try:
        odom_bridge_x = str(float(follower_x) - float(leader_x))
        odom_bridge_y = str(float(follower_y) - float(leader_y))
    except ValueError:
        odom_bridge_x = '-0.50'
        odom_bridge_y = '0.0'

    pose = [[leader_x, leader_y], [follower_x, follower_y]]

    model_folder = 'turtlebot3_' + turtlebot3_model
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    urdf_path = os.path.join(turtlebot3_gazebo_share, 'models', model_folder, 'model.sdf')
    save_dir = os.path.join('/tmp', 'escort_turtlebot_sdf', model_folder)
    os.makedirs(save_dir, exist_ok=True)

    gazebo_launch_dir = os.path.join(turtlebot3_gazebo_share, 'launch')
    escort_launch_dir = os.path.join(get_package_share_directory('escort_turtlebot_pkg'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world = os.path.join(turtlebot3_gazebo_share, 'worlds', 'turtlebot3_world.world')

    actions = []
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        )
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        )
    )
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(escort_launch_dir, 'escort_core.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'number_of_follower': '1',
            'odom_bridge_x': odom_bridge_x,
            'odom_bridge_y': odom_bridge_y,
        }.items(),
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(escort_launch_dir, 'escort_slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    robot_state_publisher_cmd_list = []
    spawn_turtlebot_cmd_list = []

    for count in range(number_of_robots):
        robot_state_publisher_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_launch_dir, 'robot_state_publisher.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'frame_prefix': f'{namespace_prefix}_{count + 1}',
                }.items(),
            )
        )

        tree = ET.parse(urdf_path)
        root = tree.getroot()
        for odom_frame_tag in root.iter('odometry_frame'):
            odom_frame_tag.text = f'{namespace_prefix}_{count + 1}/odom'
        for base_frame_tag in root.iter('robot_base_frame'):
            base_frame_tag.text = f'{namespace_prefix}_{count + 1}/base_footprint'
        for scan_frame_tag in root.iter('frame_name'):
            scan_frame_tag.text = f'{namespace_prefix}_{count + 1}/base_scan'

        urdf_modified = ET.tostring(root, encoding='unicode')
        urdf_modified = '<?xml version="1.0" ?>\n' + urdf_modified
        sdf_path = os.path.join(save_dir, f'{count + 1}.sdf')
        with open(sdf_path, 'w', encoding='utf-8') as file:
            file.write(urdf_modified)

        spawn_turtlebot_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(escort_launch_dir, 'escort_spawn.launch.py')
                ),
                launch_arguments={
                    'x_pose': str(pose[count][0]),
                    'y_pose': str(pose[count][1]),
                    'robot_name': f'{turtlebot3_model}_{count + 1}',
                    'namespace': f'{namespace_prefix}_{count + 1}',
                    'sdf_path': sdf_path,
                }.items(),
            )
        )

    actions.append(
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event, ctx: [
                    os.remove(os.path.join(save_dir, f'{count + 1}.sdf'))
                    for count in range(number_of_robots)
                    if os.path.exists(os.path.join(save_dir, f'{count + 1}.sdf'))
                ]
            )
        )
    )

    for count, spawn_turtlebot_cmd in enumerate(spawn_turtlebot_cmd_list, start=1):
        actions.append(
            GroupAction(
                [
                    PushRosNamespace(f'{namespace_prefix}_{count}'),
                    robot_state_publisher_cmd_list[count - 1],
                    spawn_turtlebot_cmd,
                ]
            )
        )

    actions.append(
        TimerAction(
            period=5.0,
            actions=[core_launch, slam_launch],
        )
    )

    return actions


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true',
        )
    )
    ld.add_action(DeclareLaunchArgument('leader_x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('leader_y', default_value='-0.5'))
    ld.add_action(DeclareLaunchArgument('follower_x', default_value='-0.5'))
    ld.add_action(DeclareLaunchArgument('follower_y', default_value='-0.5'))
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
