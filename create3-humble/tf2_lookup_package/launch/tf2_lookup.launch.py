#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(get_package_share_directory('tf2_lookup_package'),'config','tf2_lookup.yaml')
    
    return LaunchDescription([
        Node(
            package='tf2_lookup_package',
            executable='map_base_link_frame_listener',
            name='map_base_link_frame_listener',
            arguments=['--ros-args', '--log-level', 'INFO'],
            parameters=[config]
        )
    ])