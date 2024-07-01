# Copyright (c) 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    #warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    create3_dir=get_package_share_directory('create3_launch_real')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_dir = get_package_share_directory('create3_lidar_slam')
    rl_api_dir = get_package_share_directory('nav2_rl_api')
    rp_lidar_dir = get_package_share_directory('start_create3')
    # Launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')

    # Declare the launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    

    #urdf = os.path.join(irobot_urdf_dir, 'urdf', 'create3.urdf.xacro')
    

    # start the visualization
    create3_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(create3_dir, 'launch', 'create3_spawn.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'false'}.items())
    #rplidar_cmd=IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
     #       os.path.join(rp_lidar_dir, 'launch', 'rp_lidar.launch.py')))
    
    # start the automatic rl exploration 
    #rl_api = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(rl_api_dir, 'launch', 'nav2_rl.launch.py')))

    ld = LaunchDescription()
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)    
    ld.add_action(create3_cmd)
    #ld.add_action(rplidar_cmd)
    #ld.add_action(slam_cmd)
    #ld.add_action(nav_bringup_cmd)
    return ld
