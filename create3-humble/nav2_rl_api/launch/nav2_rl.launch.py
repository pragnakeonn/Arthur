#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    

    config= os.path.join(
        get_package_share_directory('nav2_rl_api'),
        'config',
       'nav2_rl_params.yaml'
       )
        
    
    """ tf2_lookup_node=Node(
            package='nav2_rl_api',
            executable='map_base_link_frame_listener',
            name='map_base_link_frame_listener',
            arguments=['--ros-args', '--log-level', 'INFO'],
            parameters=[config]
        )  """
    rl_pose_node=Node(
            package='nav2_rl_api',
            executable='explore_pose_generator',
            name='explore_pose_generator',
            arguments=['--ros-args', '--log-level', 'INFO'],
            parameters=[config]
        )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    #ld.add_action(tf2_lookup_node)    
    ld.add_action(rl_pose_node)
    
    return ld