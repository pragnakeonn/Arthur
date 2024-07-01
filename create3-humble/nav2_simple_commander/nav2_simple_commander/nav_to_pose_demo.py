#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

import time

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

import rclpy
import traceback
from rclpy.node import Node
from lfd_interfaces.msg import LfdPose
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from robot_navigator import BasicNavigator, NavigationResult

'''
Basic navigation demo to go to pose.
'''
class Nav2GoToPose(Node):
    '''
    Action class that decides when to create a waypoint
    '''
    def __init__(self):
        super().__init__('nav2_pose_action')
        self.navigator = BasicNavigator()
        self.goal_pose = PoseStamped()
        self.timer_period = 1.0
        self.posesub= self.create_subscription(PoseStamped, '/goal_pose', self.cb_pose_sub, qos_profile= 10, callback_group= ReentrantCallbackGroup())
        self.nav2_timer= self.create_timer(self.timer_period, self.cb_nav2_pose, callback_group= ReentrantCallbackGroup())
    def cb_pose_sub(self, msg):
        goto_x = msg.pose.position.x
        goto_y = msg.pose.position.y
        
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = goto_x
        self.goal_pose.pose.position.y = goto_y
        self.goal_pose.pose.orientation.w = 1.0

    def cb_nav2_pose(self):

        self.initial_pose()

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        self.navigator.lifecycleStartup()

        # Wait for navigation to fully activate, since autostarting nav2
        #navigator.waitUntilNav2Active()

        self.navigator.goToPose(self.goal_pose)

        i = 0
        while not self.navigator.isNavComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.navigator.cancelNav()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                self.goal_pose.pose.position.x = -3.0
                self.navigator.goToPose(self.goal_pose)

            # Do something depending on the return code
        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            
        elif result == NavigationResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            self.get_logger().info('Goal failed!')
        else:
            self.get_logger().info('Goal has an invalid return status!')

        self.navigator.lifecycleShutdown()

        exit(0)

        
    def initial_pose(self):
        # Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()        
        initial_pose.pose.position.x = 3.45
        initial_pose.pose.position.y = 2.15
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(initial_pose)

    

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    #get the goal pose from /lfd_next_pose
    

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    
def main(args=None):
    logger = rclpy.logging.get_logger('nav2_pose_action')
    rclpy.init(args=args)
    """ node = CreateLfdPose()
    rclpy.spin(node)
    rclpy.shutdown()  """
    try:
        create_lfd = Nav2GoToPose()
        executor = MultiThreadedExecutor(num_threads=7)
        executor.add_node(create_lfd)
        executor.spin()
       
    except Exception as e:
        logger.fatal('[nav2_pose_action]: nav2_pose_action node died')
        e = traceback.format_exc()
        logger.fatal(e)

    finally:
        executor.shutdown()
        create_lfd.destroy_node()
        rclpy.shutdown() 
  

    

if __name__ == '__main__':
    main()
