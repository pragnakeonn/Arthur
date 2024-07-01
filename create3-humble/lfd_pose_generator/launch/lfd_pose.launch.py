#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(get_package_share_directory('lfd_pose_generator'),'config','lfd_pose_generator.yaml')
    
    return LaunchDescription([
        Node(
            package='lfd_pose_generator',
            executable='lfd_pose_generator',
            name='lfd_pose_generator',
            arguments=['--ros-args', '--log-level', 'INFO'],
            parameters=[config]
        )
    ])