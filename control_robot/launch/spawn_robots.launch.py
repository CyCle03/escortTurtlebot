from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 리더 로봇 (leader)
    spawn_leader = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'leader_robot',
            '-file', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf',
            '-x', '0.5', '-y', '0.0', '-z', '0.01',
            '-robot_namespace', 'leader'
        ],
        output='screen'
    )

    # 2. 팔로워 로봇 (follower)
    spawn_follower = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'follower_robot',
            '-file', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf',
            '-x', '-0.5', '-y', '0.0', '-z', '0.01',
            '-robot_namespace', 'follower'
        ],
        output='screen'
    )

    return LaunchDescription([spawn_leader, spawn_follower])