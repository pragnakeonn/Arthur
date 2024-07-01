#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(get_package_share_directory('lfd_services'),'config','lfd_services.yaml')
    
    return LaunchDescription([
        Node(
            package='lfd_services',
            executable='lfd_services',
            name='lfd_services',
            arguments=['--ros-args', '--log-level', 'INFO'],
            parameters=[config]
        )
    ])