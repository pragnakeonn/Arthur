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
from nav2_rl_api.rl_code import DrlAgent
from lfd_interfaces.msg import LfdComp
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from lfd_interfaces.msg import LfdPose
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Bool
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
 
class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('explore_pose_generator')
        self.helper=PoseHelpers(self)      

        self.navigator = BasicNavigator()
        self.index =0
        self.navigator_success=True
        self.EXPLORATION_TIME_OUT_SEC = Duration(seconds=1200)
        self.NAV_TO_GOAL_TIMEOUT_SEC = 75
        self.DIST_THRESH_FOR_HEADING_CALC = 0.25
        self.timer_period = 1.0
        self.mapRead_flag= False
        self.current_pose_flag = False
        self.initial_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.trial_goal=PoseStamped()
        self.drl = DrlAgent()
        #self.nav2_goal_status = False
        
        self.transform= self.create_subscription(LfdPose,
            '/map_to_base_link_pose2d', self.cb_pose2d_subs, qos_profile= 10, callback_group= ReentrantCallbackGroup())

        self.map_data_subs= self.create_subscription(OccupancyGrid, '/map', self.cb_map_subs, qos_profile= 10, callback_group= ReentrantCallbackGroup())

        #self.nav2_status_sub= self.create_subscription(Bool, '/nav2_goal_status', self.cb_nav2_status, qos_profile= 10, callback_group= ReentrantCallbackGroup())
        self.explore_timer= self.create_timer(self.timer_period, self.explore, callback_group= ReentrantCallbackGroup())
      
        self.rl_pose_publisher = self.create_publisher(LfdComp, '/rl_comp_pose', 5, callback_group= ReentrantCallbackGroup())
        self.rl_img_publisher = self.create_publisher(Image, '/rl_img', 5, callback_group= ReentrantCallbackGroup())
        self.frontier_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 5, callback_group= ReentrantCallbackGroup())
        self.pose_history = list() 

        self.subscription = self.create_subscription(
            HazardDetectionVector,
            '/hazard_detection',
            self.cb_hazard_listener,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        

    def handle_hazard_msg(self, msg):
        if msg.type == msg.BUMP:
            log = self.get_logger()
            log.info('bump!', throttle_duration_sec=1)
            self.navigator.cancelNavigation()

            

    def cb_hazard_listener(self, msg):
        for hazard in msg.detections:
            self.handle_hazard_msg(hazard)
    
    def cb_map_subs(self, msg):
        self.map_data = msg.data
        self.mapData=msg
        #self.get_logger().info('Obtained map data')
        self.mapRead_flag= True

    def cb_pose2d_subs(self, msg): 
        #
        
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        self.current_pose = [self.current_x, self.current_y, self.current_z]
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = msg.pose.position.x
        self.initial_pose.pose.position.y = msg.pose.position.y
        self.initial_pose.pose.position.z = msg.pose.position.z
        self.initial_pose.pose.orientation.x = msg.pose.orientation.x
        self.initial_pose.pose.orientation.y = msg.pose.orientation.y
        self.initial_pose.pose.orientation.z = msg.pose.orientation.z
        self.initial_pose.pose.orientation.w = msg.pose.orientation.w
        #set initial pose for navigator
        
                  
        
        self.current_pose_flag = True
    

    def call_navigator(self): 
        self.get_logger().info("Navigator starts: " )
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.lifecycleStartup()


        self.get_logger().info('Path is valid')
        i = 0
        self.navigator.goToPose(self.goal_pose)
           
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
                        self.navigator.cancelNavigation()

                    # Some navigation request change to demo preemption
                    """ if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                        self.goal_pose.pose.position.x = -3.0
                        self.navigator.goToPose(self.goal_pose) """

            # Do something depending on the return code
        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            self.navigator_success=True
            self.pose_history.append([self.goal_pose.pose.position.x, self.goal_pose.pose.position.y])
        elif result == NavigationResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
            self.navigator_success=False
        elif result == NavigationResult.FAILED:
            self.get_logger().info('Goal failed!')
            self.navigator_success=False
        else:
            self.get_logger().info('Goal has an invalid return status!')
            self.navigator_success=False
        
        # once goal finished check whether this increase the map or not 
        # keep a counter if map did not increase
        

            
        #self.navigator.lifecycleShutdown()




    def explore(self):
        #self.get_logger().info("Map read flag:Comp pose flag: "+str(self.mapRead_flag)+str(self.current_pose_flag))
        if (self.mapRead_flag== True and self.current_pose_flag == True):
            if self.navigator_success==True:
                self.get_logger().info('Starting frontier exploration...')
                counter = 0
                
                self.start_time = self.get_clock().now()
                while self.start_time - self.get_clock().now() < self.EXPLORATION_TIME_OUT_SEC:
                    
                    # Delay getting next goal so map updates
                    prev_time = self.get_clock().now()
                    """ while self.get_clock().now() - prev_time < Duration(seconds=0.3):
                        self.get_logger().error('While pass condition')
                        pass """

                    # Get a frontier we can drive to
                    # call lfd 5 times and frontier once every
                    
                    status = self.get_reachable_goal(counter)
                    self.get_logger().info('rl_goal'+str(status))

                    if status == 0:
                        self.get_logger().info('Exploration complete! No frontiers detected.')
                        exit(0)
                    



                    # If we cant find a path to any frontiers
                    """ if self.goal_pose is None:
                        self.get_logger().error('No reachable frontiers!')
                        exit(-1)
                    elif self.goal_pose == "Done":
                        self.get_logger().info('Exploration complete! No frontiers detected.')
                        exit(0) """
                    
                    # Go to the goal pose
                    
                    
                    self.navigator_success=False
                    self.call_navigator()
                    if (self.navigator_success==True):
                        #self.navigator.lifecycleShutdown()
                    #self.frontier_pose_publisher.publish(self.goal_pose)
                        self.get_logger().info('Target pose complete')

                    
               

    def get_reachable_goal(self,counter):
        if self.drl:
            return self.get_reachable_goal_drl()
        else:
            return self.get_reachable_goal_cv(counter=counter)
    def check_alternating(self):
        self.get_logger().info('Check alternating condition' + str(self.pose_history)) 
        self.get_logger().info("trial goal" + 
                               str(self.trial_goal.pose.position.x) + 
                               str(self.trial_goal.pose.position.y))
        
        if len(self.pose_history)>=5:
            index = len(self.pose_history)-5
        else:
            index = 0
        last = self.pose_history[index:]
        x = self.trial_goal.pose.position.x
        y = self.trial_goal.pose.position.y
        dists = []
        for l in last:
            x1 = l[0]
            y1 = l[1]
            dist = np.sqrt((x - x1)**2 + (y - y1)**2)
            dists.append(dist)
        self.get_logger().info("dist to previous goals" + str(dists)) 
        
        if any([dist <0.5 for dist in dists]) and len(dists) >0:
            self.get_logger().info("alternating goal") 

            return True
        else: 
            self.get_logger().info("not alternating goal")
            return False
        
    def get_reachable_goal_cv(self, counter):
        rank = 0
        reachable = False
        
        while not reachable:
                
                goals, img = getfrontier(mapData=self.mapData, curr_pos= self.current_pose)
                # check if path possible here
                best_goal = None
                for goal in goals:
                    self.trial_goal.header.frame_id = 'map'
                    self.trial_goal.header.stamp = self.get_clock().now().to_msg()
                    trial_pose=[goal[0], goal[1], self.initial_pose.pose.position.z]
                    self.next_quat_x, self.next_quat_y, self.next_quat_z, self.next_quat_w = self.helper.rot2quatarnion(current_pose=self.current_pose, next_pose=trial_pose)   
                    
                    self.trial_goal.pose.position.x = goal[0]
                    self.trial_goal.pose.position.y = goal[1]
                    self.trial_goal.pose.position.z = self.initial_pose.pose.position.z
                    self.trial_goal.pose.orientation.x = self.initial_pose.pose.orientation.x
                    self.trial_goal.pose.orientation.y = self.initial_pose.pose.orientation.y
                    self.trial_goal.pose.orientation.z = self.initial_pose.pose.orientation.z
                    self.trial_goal.pose.orientation.w = self.initial_pose.pose.orientation.w
                    # if not self.check_alternating():
                    if self.navigator.getPath(self.initial_pose, self.trial_goal) is not None:
                        self.get_logger().info('Path found'+str(goal))
                        best_goal = goal
                        break
                if best_goal is not None:
                #self.get_logger().info('Image frontier'+str(img.shape))
                #(82, 74, 1)
                    goal_z = self.initial_pose.pose.position.z 
                    rl_goal_pose = [best_goal[0], best_goal[1], goal_z]

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
                
                #self.rl_img_publisher.publish(frontier_image)
            
                if best_goal is None:
                    status = 0
                    return status
            
            


                self.next_quat_x, self.next_quat_y, self.next_quat_z, self.next_quat_w = self.helper.rot2quatarnion(current_pose=self.current_pose, next_pose=rl_goal_pose)
                self.goal_pose.header.frame_id = 'map'
                self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    
                self.goal_pose.pose.position.x = best_goal[0]
                self.goal_pose.pose.position.y = best_goal[1]
                self.goal_pose.pose.position.z = goal_z
                self.goal_pose.pose.orientation.x =  self.next_quat_x
                self.goal_pose.pose.orientation.y =  self.next_quat_y
                self.goal_pose.pose.orientation.z =  self.next_quat_z
                self.goal_pose.pose.orientation.w =  self.next_quat_w
                status = 1

                
                reachable = True

                rank += 1
        return status
    def get_reachable_goal_drl(self):
        rank = 0
        reachable = False
        
        while not reachable:
                
                goals,img = self.drl.forward(mapData=self.mapData, curr_pos= self.current_pose)
                # check if path possible here
                best_goal = None
                self.get_logger().info('Path checking'+str(goals))
                for goal in goals:
                    self.trial_goal.header.frame_id = 'map'
                    self.trial_goal.header.stamp = self.get_clock().now().to_msg()
                    self.get_logger().info('Path checking'+str(goal)) 
                    rl_goal_pose= [float(goal[0]), float(goal[1]), self.initial_pose.pose.position.z]
                    self.next_quat_x, self.next_quat_y, self.next_quat_z, self.next_quat_w = self.helper.rot2quatarnion(current_pose=self.current_pose, next_pose=rl_goal_pose) 
                    self.trial_goal.pose.position.x = float(goal[0])
                    self.trial_goal.pose.position.y = float(goal[1])
                    self.trial_goal.pose.position.z = self.initial_pose.pose.position.z
                    self.trial_goal.pose.orientation.x =  self.next_quat_x
                    self.trial_goal.pose.orientation.y =  self.next_quat_y
                    self.trial_goal.pose.orientation.z =  self.next_quat_z
                    self.trial_goal.pose.orientation.w =  self.next_quat_w
                    # if not self.check_alternating():
                    if self.navigator.getPath(self.initial_pose, self.trial_goal) is not None:
                        self.get_logger().info('Path found'+str(goal))
                        best_goal = goal
                        break
                if best_goal is not None:
                #self.get_logger().info('Image frontier'+str(img.shape))
                #(82, 74, 1)
                    goal_z = self.initial_pose.pose.position.z 
                    rl_goal_pose = [best_goal[0], best_goal[1], goal_z]

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
                
                #self.rl_img_publisher.publish(frontier_image)
            
                if best_goal is None:
                    status =1
                    return status
            
            


                self.next_quat_x, self.next_quat_y, self.next_quat_z, self.next_quat_w = self.helper.rot2quatarnion(current_pose=self.current_pose, next_pose=rl_goal_pose)
                self.goal_pose.header.frame_id = 'map'
                self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    
                self.goal_pose.pose.position.x = best_goal[0]
                self.goal_pose.pose.position.y = best_goal[1]
                self.goal_pose.pose.position.z = goal_z
                self.goal_pose.pose.orientation.x =  self.next_quat_x
                self.goal_pose.pose.orientation.y =  self.next_quat_y
                self.goal_pose.pose.orientation.z =  self.next_quat_z
                self.goal_pose.pose.orientation.w =  self.next_quat_w

                
                reachable = True
                status = 1

                rank += 1
                
        return status


def main(args=None):
    logger = rclpy.logging.get_logger('explore_pose_generator')
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown() 
    """ try:
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
   """
if __name__ == '__main__':
  main()