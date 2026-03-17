from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import os

def generate_launch_description():
    print(f"PYTHONPATH: {os.environ.get('PYTHONPATH', 'NOT SET')}")
    print(f"sys.path: {sys.path}")
    
    return LaunchDescription([
        Node(
            package='escort_turtlebot_pkg',
            executable='escort_vision_node',
            name='debug_node',
            output='screen',
            env={'PYTHONPATH': os.environ.get('PYTHONPATH', '')}
        )
    ])
