#! /usr/bin/env python3

'''
This node is for autonomous exploration. It requests frontier regions from a service and
sends the goal points to the nav2 stack until the the entire environment has been explored.

Reference code: https://automaticaddison.com/how-to-send-goals-to-the-ros-2-navigation-stack-nav2/
'''
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_rl_api.lib.helper import PoseHelpers
import traceback
from rclpy.executors import MultiThreadedExecutor
from nav2_rl_api.robot_navigator import BasicNavigator, NavigationResult
from nav2_rl_api.getFrontiers import getfrontier
from lfd_interfaces.msg import LfdComp
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from lfd_interfaces.msg import LfdPose
from sensor_msgs.msg import Image
import numpy as np
 
class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('rl_pose_generator')
        self.helper=PoseHelpers(self)      

        self.navigator = BasicNavigator()

        self.EXPLORATION_TIME_OUT_SEC = Duration(seconds=1200)
        self.NAV_TO_GOAL_TIMEOUT_SEC = 75
        self.DIST_THRESH_FOR_HEADING_CALC = 0.25
        self.timer_period = 1.0
        self.mapRead_flag= False
        self.compRead_flag = False
        self.initial_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        
        self.lfd_comp_pose= self.create_subscription(LfdComp, '/comp_pose', self.cb_comp_subs, qos_profile= 10, callback_group= ReentrantCallbackGroup())

        self.map_data_subs= self.create_subscription(OccupancyGrid, '/map', self.cb_map_subs, qos_profile= 10, callback_group= ReentrantCallbackGroup())
       
        self.explore_timer= self.create_timer(self.timer_period, self.explore, callback_group= ReentrantCallbackGroup())
      
        self.rl_pose_publisher = self.create_publisher(LfdPose, '/rl_pose', 5, callback_group= ReentrantCallbackGroup())
        self.rl_img_publisher = self.create_publisher(Image, '/rl_img', 5, callback_group= ReentrantCallbackGroup())
    def cb_map_subs(self, msg):
        self.map_data = msg.data
        self.mapData=msg
        self.get_logger().info('Obtained map data')
        self.mapRead_flag= True

    def cb_comp_subs(self, compmsg):
        self.current_x = compmsg.currpose.position.x
        self.current_y = compmsg.currpose.position.y
        self.current_z = compmsg.currpose.position.z
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = compmsg.currpose.position.x
        self.initial_pose.pose.position.y = compmsg.currpose.position.y
        self.initial_pose.pose.position.z = compmsg.currpose.position.z
        self.initial_pose.pose.orientation.x = compmsg.currpose.orientation.x
        self.initial_pose.pose.orientation.y = compmsg.currpose.orientation.y
        self.initial_pose.pose.orientation.z = compmsg.currpose.orientation.z
        self.initial_pose.pose.orientation.w = compmsg.currpose.orientation.w
        self.current_pose = [self.current_x, self.current_y, self.current_z]
        self.navigator.setInitialPose(self.initial_pose)
        #self.get_logger().info("Initial_pose_created: " + 'x: ' + str(self.current_x)+" "+"y: " + str(self.current_y)+ " "+"z: " + str(self.current_z))


        self.goal_pose.header.frame_id = 'map'        
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()      
        self.goal_pose.pose.position.x = compmsg.nextpose.position.x
        self.goal_pose.pose.position.y = compmsg.nextpose.position.x
        self.goal_pose.pose.position.z = compmsg.nextpose.position.x
        self.goal_pose.pose.orientation.x = compmsg.nextpose.orientation.x
        self.goal_pose.pose.orientation.y = compmsg.nextpose.orientation.x
        self.goal_pose.pose.orientation.z = compmsg.nextpose.orientation.x
        self.goal_pose.pose.orientation.w = compmsg.nextpose.orientation.x
        self.next_x = compmsg.nextpose.position.x
        self.next_y = compmsg.nextpose.position.y
        self.next_z = compmsg.nextpose.position.z
        self.next_pose = [self.next_x, self.next_y, self.next_z]

        self.get_logger().info("Goal_pose_created: " )
        
        self.compRead_flag = True

    def explore(self):
        self.get_logger().info("Map read flag:Comp pose flag: "+str(self.mapRead_flag)+str(self.compRead_flag))
        if (self.mapRead_flag== True and self.compRead_flag == True):
            self.get_logger().info('Starting frontier exploration...')
            counter = 0
            
            self.start_time = self.get_clock().now()
            while self.start_time - self.get_clock().now() < self.EXPLORATION_TIME_OUT_SEC:

                # Delay getting next goal so map updates
                prev_time = self.get_clock().now()
                while self.get_clock().now() - prev_time < Duration(seconds=0.3):
                    pass

                # Get a frontier we can drive to
                # call lfd 5 times and frontier once every
                
                rl_goal = self.get_reachable_goal(counter)
                

                # If we cant find a path to any frontiers
                if self.goal_pose is None:
                    self.get_logger().error('No reachable frontiers!')
                    exit(-1)
                elif self.goal_pose == "Done":
                    self.get_logger().info('Exploration complete! No frontiers detected.')
                    exit(0)
                
                # Go to the goal pose
                """ self.navigator.goToPose(self.goal_pose)
                
                # Keep doing stuff as long as the robot is moving towards the goal
                i = 0
                while not self.navigator.isNavigationComplete():

                    i = i + 1
                    feedback = self.navigator.getFeedback()
                    if feedback and i % 40 == 0:
                        self.get_logger().info('Distance remaining: ' + '{:.2f}'.format(
                            feedback.distance_remaining) + ' meters.')
                    
                        # Set goal pose heading to robots current heading once it is close]
                        # This avoids unecessary rotation once reaching the goal position
                        if feedback.distance_remaining <= self.DIST_THRESH_FOR_HEADING_CALC:
                            self.get_logger().info('Setting new heading')
                            self.set_goal_heading()
                            self.navigator.goToPose(self.goal_pose)
                
                    # Cancel the goal if robot takes too long
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=self.NAV_TO_GOAL_TIMEOUT_SEC):
                        self.navigator.cancelNav()
                
                # Print result when nav completes
                result = self.navigator.getResult()
                self.log_nav_status(result) """

                   
                rl_pose = LfdPose()
                rl_pose.header.frame_id = 'map'        
                rl_pose.header.stamp = self.navigator.get_clock().now().to_msg()      
                rl_pose.pose.position.x = rl_goal[0]
                rl_pose.pose.position.y = rl_goal[1]
                rl_pose.pose.position.z = rl_goal[2]
                rl_pose.pose.orientation.x = self.goal_pose.pose.orientation.x 
                rl_pose.pose.orientation.y = self.goal_pose.pose.orientation.y 
                rl_pose.pose.orientation.z = self.goal_pose.pose.orientation.z 
                rl_pose.pose.orientation.w = self.goal_pose.pose.orientation.w 

                self.rl_pose_publisher.publish(rl_pose)

                """ rl_img = Float32MultiArray()
                rl_img.data = rl_img
                self.rl_img_publisher.publish(rl_img) """



    def get_reachable_goal(self, counter):
        rank = 0
        reachable = False
        
        while not reachable:
            if counter %6 == 0:
                goal, img = getfrontier(mapData=self.mapData, curr_pos= self.current_pose)
                self.get_logger().info('Image frontier'+str(img.shape))
                #(82, 74, 1)
                goal_z = self.goal_pose.pose.position.z 
                rl_goal_pose = [goal[0], goal[1], goal_z]

                frontier_image = Image()
                #frontier_image.header= 'map'
                frontier_image.header.stamp = self.get_clock().now().to_msg()
                frontier_image.header.frame_id = 'map'
                frontier_image.height=img.shape[0]
                frontier_image.width=img.shape[1]
                frontier_image.step=img.shape[1] 
                frontier_image.is_bigendian = False
                frontier_image.encoding = 'mono8'
                frontier_image.data=np.array(img).tobytes()
                
                self.rl_img_publisher.publish(frontier_image)
            else:
                rl_goal_pose = self.next_pose
            if goal is None:
                return "Done"
            
            


            self.next_quat_x, self.next_quat_y, self.next_quat_z, self.next_quat_w = self.helper.rot2quatarnion(current_pose=self.current_pose, next_pose=rl_goal_pose)
            self.goal_pose.header.frame_id = 'map'
            self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.goal_pose.header.frame_id = 'map'        
            self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()      
            self.goal_pose.pose.position.x = goal[0]
            self.goal_pose.pose.position.y = goal[1]
            self.goal_pose.pose.position.z = goal_z
            self.goal_pose.pose.orientation.x = self.next_quat_x
            self.goal_pose.pose.orientation.y = self.next_quat_y
            self.goal_pose.pose.orientation.z = self.next_quat_z
            self.goal_pose.pose.orientation.w = self.next_quat_w

            # sanity check a valid path exists
            
            """ if self.initial_pose is None:
                # Return goal is current pose is unavailble
                rl_goal_pose = [0,0,0]
                return rl_goal_pose
            path = self.navigator.getPath(self.initial_pose, self.goal_pose)

            # If top 4 frontiers are not reachable, abort
            if path is not None:
                return rl_goal_pose
            elif rank > 3:
                return None """
            rank += 1


def main(args=None):
    logger = rclpy.logging.get_logger('rl_pose_generator')
    rclpy.init(args=args)
    """ node = FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()  """
    try:
        create_rl = FrontierExplorer()
        executor = MultiThreadedExecutor(num_threads=9)
        executor.add_node(create_rl)
        executor.spin()
       
    except Exception as e:
        logger.fatal('[rl_pose_generator]: RL generator node died')
        e = traceback.format_exc()
        logger.fatal(e)

    finally:
        executor.shutdown()
        create_rl.destroy_node()
        rclpy.shutdown() 
  
if __name__ == '__main__':
  main()