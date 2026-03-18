import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    vision_node = Node(
        package='escort_turtlebot_pkg',
        executable='escort_vision_node',
        name='escort_vision_node',
        output='screen',
        parameters=[{'model_path': '/home/penguin/escort_ws/yolo_model/last_openvino_model'}]
    )

    gui_node = Node(
        package='escort_turtlebot_pkg',
        executable='escort_turtlebot_gui',
        name='escort_turtlebot_gui',
        output='screen'
    )

    return LaunchDescription([
        vision_node,
        gui_node
    ])
