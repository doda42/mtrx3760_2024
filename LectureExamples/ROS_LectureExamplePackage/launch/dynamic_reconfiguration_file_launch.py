from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param = DeclareLaunchArgument('param_config',
                                  default_value='/home/khit/MTRX3760/src/mtrx3760/config/dynamic_reconfiguration_params.yaml')
    node = Node(package='mtrx3760',
                executable='nodeWithParam',
                parameters=[LaunchConfiguration('param_config')])
    return LaunchDescription([param, node])
