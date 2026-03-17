from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    vision_node = Node(
        package='escort_turtlebot_pkg',
        executable='escort_vision_node',
        name='escort_vision_node',
        output='screen',
        parameters=[{'model_path': 'yolo_model/best.pt'}],
        env={
            'QT_QPA_PLATFORM_PLUGIN_PATH': '/usr/lib/x86_64-linux-gnu/qt5/plugins/',
            'QT_DEBUG_PLUGINS': '1'
        }
    )

    gui_node = Node(
        package='escort_turtlebot_pkg',
        executable='escort_turtlebot_gui',
        name='escort_turtlebot_gui',
        output='screen',
        env={
            'QT_QPA_PLATFORM_PLUGIN_PATH': '/usr/lib/x86_64-linux-gnu/qt5/plugins/',
            'QT_DEBUG_PLUGINS': '1'
        }
    )

    return LaunchDescription([
        vision_node,
        gui_node
    ])
