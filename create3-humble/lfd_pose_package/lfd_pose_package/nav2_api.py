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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from lfd_interfaces.msg import LfdComp  
import traceback
from std_msgs.msg import Bool
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
"""
Basic navigation demo to go to pose.
"""
class Nav2api(Node):
    '''
    Action class that decides when to create a waypoint
    '''
    def __init__(self):
        super().__init__('nav2_api')
        self.goal_statuse=False
        self.initial_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.navigator = BasicNavigator()       
        self.timer_period = 1.0
        # Init laserscan subscription 
        self.lfd_comp_pose= self.create_subscription(LfdComp, '/comp_pose', self.cb_comp_subs, qos_profile= 10, callback_group= ReentrantCallbackGroup())
        self.goal_status_pub=self.create_publisher(Bool, '/nav2_goal_status', 5, callback_group= ReentrantCallbackGroup())

    def cb_comp_subs(self, msg):
        self.get_logger().info("Comp msg read by nav2_api: " )

        # Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = msg.currpose.position.x
        initial_pose.pose.position.y = msg.currpose.position.y
        initial_pose.pose.position.z = msg.currpose.position.z
        initial_pose.pose.orientation.x = msg.currpose.orientation.x
        initial_pose.pose.orientation.y = msg.currpose.orientation.y
        initial_pose.pose.orientation.z = msg.currpose.orientation.z
        initial_pose.pose.orientation.w = msg.currpose.orientation.w
        self.navigator.setInitialPose(initial_pose)

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        self.navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
        #self.navigator.waitUntilNav2Active()

        # If desired, you can change or load the map as well
        # navigator.changeMap('/path/to/map.yaml')

        # You may use the navigator to clear or obtain costmaps
        # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()

        # Go to our demos first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.nextpose.position.x
        goal_pose.pose.position.y = msg.nextpose.position.y
        goal_pose.pose.position.z = msg.nextpose.position.z
        goal_pose.pose.orientation.x = msg.nextpose.orientation.x
        goal_pose.pose.orientation.y = msg.nextpose.orientation.y
        goal_pose.pose.orientation.z = msg.nextpose.orientation.z
        goal_pose.pose.orientation.w = msg.nextpose.orientation.w

        # sanity check a valid path exists
        path = self.navigator.getPath(initial_pose, goal_pose)

        self.navigator.goToPose(goal_pose)
        if (path!=None):
            self.get_logger().info('Path is valid')
            i = 0
            while not self.navigator.isNavigationComplete():
                ################################################
                #
                # Implement some code here for your application!
                #
                ################################################

                # Do something with the feedback
                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time of arrival: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')

                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        self.navigator.cancelTask()

                    # Some navigation request change to demo preemption
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                        goal_pose.pose.position.x = -3.0
                        self.navigator.goToPose(goal_pose)

            # Do something depending on the return code
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
                self.goal_status=Bool()
                self.goal_status=True
                self.goal_status_pub.publish(self.goal_status)

                
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

            #self.navigator.lifecycleShutdown()

            #exit(0)

def main(args=None):
    logger = rclpy.logging.get_logger('nav2_api')
    rclpy.init(args=args)
    """ node = CreateLfdPose()
    rclpy.spin(node)
    rclpy.shutdown()  """
    try:
        nav2_api = Nav2api()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(nav2_api)
        executor.spin()
       
    except Exception as e:
        logger.fatal('[nav2_api]: Nav2 Api node died')
        e = traceback.format_exc()
        logger.fatal(e)

    finally:
        executor.shutdown()
        nav2_api.destroy_node()
        rclpy.shutdown() 
if __name__ == '__main__':
    main()
