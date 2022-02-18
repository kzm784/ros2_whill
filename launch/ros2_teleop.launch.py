import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        remappings=[('joy', '/whill/controller/joy')])
    
    ld = LaunchDescription()

    ld.add_action(joy_node)

    return ld