#!/usr/bin/env python3
import rclpy
import traceback
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from lfd_interfaces.srv import LfdRes
from lfd_interfaces.msg import LfdPose, LfdComp
from lfd_pose_package.lib.helper_class import PoseHelpers

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rclpy.duration import Duration
# Stores known frames and offers frame graph requests

from lfd_pose_package.robot_navigator import BasicNavigator, NavigationResult

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import datetime


class CreateLfdPose(Node):
    '''
    Action class that decides when to create a waypoint
    '''
    def __init__(self, SMC_flag):
        super().__init__('lfd_pose_generator')
        self.pose_helpers = PoseHelpers(self)
        self.SMC_flag = SMC_flag
        self.current_pose_flag = False
        self.rl_pose_flag = False
        self.current_angle_flag = False
        self.nav2_goal_status=False
        self.lfd_srv_futures = []
        self.incomplete_futures = []
        self.current_poses = []
        self.angles = []
        self.curr_index = 0
        
        
        self.wp_completed_msg = None
        self.curr_pose=list()
        self.next_pose = list()
        self.initial_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.rl_goal_pose = PoseStamped()


       
        self.timer_period = 1.0

        
        # Init laserscan subscription 
        self.lasersub= self.create_subscription(LaserScan, 
        '/scan', self.cb_laserscan_subs, qos_profile= 10, callback_group= ReentrantCallbackGroup())
        #Initi tf pose subscription
        self.transform= self.create_subscription(LfdPose,
            '/map_to_base_link_pose2d', self.cb_pose2d_subs, qos_profile= 10, callback_group= ReentrantCallbackGroup())
        
        self.rl_pose_subs= self.create_subscription(PoseStamped,
            '/rl_pose', self.cb_rlpose_subs, qos_profile= 10, callback_group= ReentrantCallbackGroup())
        
        
        #self.nav2_status_sub= self.create_subscription(Bool, '/nav2_goal_status', self.cb_nav2_status, qos_profile= 10, callback_group= ReentrantCallbackGroup())
        #lfd service client
        self.cli_lfdsrv = self.create_client(LfdRes, '/lfd_srv', callback_group= ReentrantCallbackGroup())
        while not self.cli_lfdsrv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service LFD not available, waiting again...')
        self.req = LfdRes.Request()     
        
        self.srv_timer= self.create_timer(self.timer_period, self.cb_request_srv, callback_group= ReentrantCallbackGroup())
        
        
        self.comp_pose_publisher = self.create_publisher(LfdComp, '/comp_pose', 5, callback_group= ReentrantCallbackGroup())
        
            
    def cb_rlpose_subs(self, msg):
        self.rl_x = msg.pose.position.x
        self.rl_y = msg.pose.position.y
        self.rl_z = msg.pose.position.z
        self.rl_current_pose = [self.rl_x, self.rl_y, self.rl_z]
        self.rl_goal_pose.header.frame_id = 'map'
        self.rl_goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.rl_goal_pose.pose.position.x = msg.pose.position.x
        self.rl_goal_pose.pose.position.y = msg.pose.position.y
        self.rl_goal_pose.pose.position.z = msg.pose.position.z
        self.rl_goal_pose.pose.orientation.x = msg.pose.orientation.x
        self.rl_goal_pose.pose.orientation.y = msg.pose.orientation.y
        self.rl_goal_pose.pose.orientation.z = msg.pose.orientation.z
        self.rl_goal_pose.pose.orientation.w = msg.pose.orientation.w
        self.rl_pose_flag=True

    def cb_pose2d_subs(self, msg): 
        #
        
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        self.current_pose_2d = [self.current_x, self.current_y, self.current_z]
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
        #self.get_logger().info("Initial_pose_created: " + 'x: ' + str(self.current_x)+" "+"y: " + str(self.current_y)+ " "+"z: " + str(self.current_z))
    
    
    def lfd_pose_pub(self):
        """ lfd_pose = LfdPose()
        lfd_pose.header.frame_id = 'map'        
        lfd_pose.header.stamp = self.get_clock().now().to_msg()   
        lfd_pose.pose.position.x = self.goal_pose.pose.position.x 
        lfd_pose.pose.position.y = self.goal_pose.pose.position.y 
        lfd_pose.pose.position.z = self.goal_pose.pose.position.z 
        lfd_pose.pose.orientation.x = self.goal_pose.pose.orientation.x 
        lfd_pose.pose.orientation.y = self.goal_pose.pose.orientation.y 
        lfd_pose.pose.orientation.z = self.goal_pose.pose.orientation.z 
        lfd_pose.pose.orientation.w = self.goal_pose.pose.orientation.w 
 """
        lfd_comp_msg = LfdComp()
        lfd_comp_msg.header.frame_id = 'map'  

        lfd_comp_msg.header.stamp = self.get_clock().now().to_msg()
        lfd_comp_msg.currpose.position.x = self.initial_pose.pose.position.x
        lfd_comp_msg.currpose.position.y = self.initial_pose.pose.position.y
        lfd_comp_msg.currpose.position.z = self.initial_pose.pose.position.z
        lfd_comp_msg.currpose.orientation.x = self.initial_pose.pose.orientation.x
        lfd_comp_msg.currpose.orientation.y = self.initial_pose.pose.orientation.y
        lfd_comp_msg.currpose.orientation.z = self.initial_pose.pose.orientation.z
        lfd_comp_msg.currpose.orientation.w = self.initial_pose.pose.orientation.w

        lfd_comp_msg.nextpose.position.x = self.goal_pose.pose.position.x 
        lfd_comp_msg.nextpose.position.y = self.goal_pose.pose.position.y
        lfd_comp_msg.nextpose.position.z = self.goal_pose.pose.position.z
        lfd_comp_msg.nextpose.orientation.x = self.goal_pose.pose.orientation.x
        lfd_comp_msg.nextpose.orientation.y = self.goal_pose.pose.orientation.z
        lfd_comp_msg.nextpose.orientation.z = self.goal_pose.pose.orientation.z
        lfd_comp_msg.nextpose.orientation.w = self.goal_pose.pose.orientation.w

        self.comp_pose_publisher.publish(lfd_comp_msg)
        #self.get_logger().info('Curr index'+str(self.curr_index))
        """ if (self.curr_index==0):
            #self.lfd_pose_publisher.publish(lfd_pose)
            self.comp_pose_publisher.publish(lfd_comp_msg)
            self.get_logger().info('Nav2 goal pose published for index'+str(self.curr_index))
        elif (self.curr_index>0):
            if(self.nav2_goal_status==True):
                #self.lfd_pose_publisher.publish(lfd_pose)
                self.comp_pose_publisher.publish(lfd_comp_msg)
                self.get_logger().info('Nav2 goal pose published for index'+str(self.curr_index)) """


    """ def cb_nav2_status(self, msg):
        self.nav2_goal_status=msg.data
        self.get_logger().info('Nav2 goal status obtained')
 """
    def send_request(self):
        now = datetime.datetime.now()
        self.get_logger().info("[Request Lfd service]: " + str(now.day)+"," + str(now.hour)+ ":" + str(now.minute)+ " - Trying to provide lfd service...")
        #self.req.angle = self.angle
        self.req.angle = self.angles[-1]
        self.req.pose.data = [self.current_x, self.current_y, self.current_z]
        #self.lfd_srv_futures.append(self.cli_lfdsrv.call_async(self.req))
        self.future = self.cli_lfdsrv.call_async(self.req)
        #rclpy.spin_until_future_complete(self, self.future)
        while not self.future.done(): pass
        return self.future.result()
        
        #self.print_periodic_upload_info(future.result())
    def cb_laserscan_subs(self, msg):
        #self.get_logger().info('I heard angle "%f"' %msg.angle_min )
        self.angle, self.range = self.pose_helpers.get_direction(laser_scan=msg)
        self.current_angle_flag=True
        #self.get_logger().info('inside scan listener callback,current angle: ' + str(self.angle)+' '+'range: ' + str(self.range))
        self.angles.append(self.angle)
        #self.get_logger().info('inside scan listener callback, last angle value ' + str(self.angles[-1]))

    def cb_request_srv(self):
        #self.get_logger().info('Current pose and angle flag '+ str(self.current_pose_flag)+'self.current_angle_flag')
        if (self.current_pose_flag==True and self.current_angle_flag==True):
            self.get_logger().info('Current pose and angle obtained '+ str(self.current_pose_2d)+str(self.angle))
            response = self.send_request()
            #self.get_logger().info('Client send request returned '+ str(response))
            if (response):
                    if (self.rl_pose_flag==False):
                        self.get_logger().info('LFD Client send request returned')

                        self.next_pose = response.nextpose.data
                        self.next_pose[0] = self.next_pose[0]
                        self.next_pose[1] = self.next_pose[1]
                        self.next_pose[2] = 0.0
                        #self.next_pose[2] = self.next_pose[2]
                        #self.get_logger().info('Request Received, next_pose' + str(self.next_pose[0]))
                        #self.get_logger().info('Request Received, next_pose' + str(self.next_pose[1]))
                        #self.get_logger().info('Request Received, next_pose' + str(self.next_pose[2])) 
                        obtained_distance = self.pose_helpers.get_euclidean_distance(curr_pose=self.current_pose_2d, next_pose=self.next_pose)       
                        if (obtained_distance<=1.0):
                            self.get_logger().info('Distance is less than 1.0' + str(obtained_distance))
                            self.next_pose[0] = self.next_pose[0]*0.5
                            self.next_pose[1] = self.next_pose[1]*0.5
                            self.next_pose[2] = 0.0
    
                        self.next_quat_x, self.next_quat_y, self.next_quat_z, self.next_quat_w = self.pose_helpers.rot2quatarnion(current_pose=self.current_pose_2d, next_pose=self.next_pose)
                        
                        self.goal_pose.header.frame_id = 'map'        
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()     
                        self.goal_pose.pose.position.x = self.next_pose[0]
                        self.goal_pose.pose.position.y = self.next_pose[1]
                        self.goal_pose.pose.position.z = self.next_pose[2]
                        self.goal_pose.pose.orientation.x =self.initial_pose.pose.orientation.x
                        self.goal_pose.pose.orientation.y =self.initial_pose.pose.orientation.y
                        self.goal_pose.pose.orientation.z =self.initial_pose.pose.orientation.z
                        self.goal_pose.pose.orientation.w =self.initial_pose.pose.orientation.w
                        """ self.goal_pose.pose.orientation.x = self.next_quat_x
                        self.goal_pose.pose.orientation.y = self.next_quat_y
                        self.goal_pose.pose.orientation.z = self.next_quat_z
                        self.goal_pose.pose.orientation.w = self.next_quat_w """

                        self.lfd_pose_pub()
                        self.curr_index =  self.curr_index + 1
                           
                            
                    elif (self.rl_pose_flag==True):
                        self.get_logger().info('RL has returned new goal pose')
                                                                      

                        self.next_pose = self.rl_current_pose
                        #self.next_pose[2] = self.next_pose[2]
                        self.get_logger().info('Request Received, next_pose' + str(self.next_pose[0]))
                        self.get_logger().info('Request Received, next_pose' + str(self.next_pose[1]))
                        self.get_logger().info('Request Received, next_pose' + str(self.next_pose[2])) 
                        obtained_distance = self.pose_helpers.get_euclidean_distance(curr_pose=self.current_pose_2d, next_pose=self.next_pose)       
                        if (obtained_distance<=1.0):
                            self.get_logger().info('Distance is less than 1.0' + str(obtained_distance))
                            self.next_pose[0] = self.next_pose[0]*0.5
                            self.next_pose[1] = self.next_pose[1]*0.5
                            self.next_pose[2] = 0.0
    
                        self.next_quat_x, self.next_quat_y, self.next_quat_z, self.next_quat_w = self.pose_helpers.rot2quatarnion(current_pose=self.current_pose_2d, next_pose=self.next_pose)
                        
                        self.goal_pose.header.frame_id = 'map'        
                        self.goal_pose.header.stamp = self.get_clock().now().to_msg()    
                        self.goal_pose.pose.position.x = self.next_pose[0]
                        self.goal_pose.pose.position.y = self.next_pose[1]
                        self.goal_pose.pose.position.z = self.next_pose[2]
                        self.goal_pose.pose.orientation.x =self.initial_pose.pose.orientation.x
                        self.goal_pose.pose.orientation.y =self.initial_pose.pose.orientation.y
                        self.goal_pose.pose.orientation.z =self.initial_pose.pose.orientation.z
                        self.goal_pose.pose.orientation.w =self.initial_pose.pose.orientation.w
                        """ self.goal_pose.pose.orientation.x = self.next_quat_x
                        self.goal_pose.pose.orientation.y = self.next_quat_y
                        self.goal_pose.pose.orientation.z = self.next_quat_z
                        self.goal_pose.pose.orientation.w = self.next_quat_w
                         """
                        
                    
                        self.lfd_pose_pub()
                        self.curr_index =  self.curr_index + 1
    
def main(args=None):
    logger = rclpy.logging.get_logger('lfd_pose_generator')
    rclpy.init(args=args)
    """ node = CreateLfdPose()
    rclpy.spin(node)
    rclpy.shutdown()  """
    try:
        create_lfd = CreateLfdPose(SMC_flag=False)
        executor = MultiThreadedExecutor(num_threads=9)
        executor.add_node(create_lfd)
        executor.spin()
       
    except Exception as e:
        logger.fatal('[lfd_pose_generator]: Lfd generator node died')
        e = traceback.format_exc()
        logger.fatal(e)

    finally:
        executor.shutdown()
        create_lfd.destroy_node()
        rclpy.shutdown() 
  
if __name__ == '__main__':
  main()