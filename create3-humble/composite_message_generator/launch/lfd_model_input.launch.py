#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(get_package_share_directory('composite_message_generator'),'config','tf2_lookup.yaml')
    
    return LaunchDescription([
        Node(
            package='composite_message_generator',
            executable='lfd_model_input',
            name='lfd_model_input',
            arguments=['--ros-args', '--log-level', 'INFO'],
            parameters=[config]
        )
    ])