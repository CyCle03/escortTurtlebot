from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    vision_node = ExecuteProcess(
        cmd=[
            'python3',
            '/home/ubuntu/robot_ws/src/escort_robot/escort_vision_node.py'
        ],
        output='screen'
    )

    gui_node = ExecuteProcess(
        cmd=[
            'python3',
            '/home/ubuntu/robot_ws/src/escort_robot/escort_turtlebot_gui.py'
        ],
        output='screen'
    )

    return LaunchDescription([
        vision_node,
        gui_node
    ])
