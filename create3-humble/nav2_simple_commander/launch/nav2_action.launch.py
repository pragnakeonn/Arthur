#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    #config = os.path.join(get_package_share_directory('nav2_gotopose_api'),'config','nav2_action_params.yaml')
    
    return LaunchDescription([
        Node(
            package='nav2_simple_commander',
            executable='nav2_pose_action',
            name='nav2_pose_action',
            arguments=['--ros-args', '--log-level', 'INFO']
            
        )
    ])