from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
       Node(
           package='mtrx3760',
           executable='simplePublisher',
       ),
       Node(
           package='mtrx3760',
           executable='simpleSubscriber',
       )
   ])
