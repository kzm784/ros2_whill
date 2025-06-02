# Copyright (c) 2024 WHILL, Inc.
# Released under the MIT license
# https://opensource.org/licenses/mit-license.php

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    model_file = os.path.join(
        get_package_share_directory('whill_description'),
        'urdf',
        'whill_model_cr2.urdf'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': open(model_file, 'r').read()
            }],
            remappings=[('/joint_states', '/whill/joint_states')]
        ),

        Node(
            package='whill_driver',
            executable='whill',
            name='whill',
            parameters=[PathJoinSubstitution([
                FindPackageShare('whill_bringup'),
                'config',
                'params.yaml'
            ])],
        ),
    ])
