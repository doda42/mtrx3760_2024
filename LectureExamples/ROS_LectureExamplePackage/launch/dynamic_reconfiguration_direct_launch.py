from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(package='mtrx3760',
                executable='nodeWithParam',
                parameters=[{'speed': 40}])
    return LaunchDescription([node])
