#!/usr/bin/env python3
import rclpy
import traceback
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from lfd_interfaces.srv import SMCRes
from lfd_interfaces.msg import LfdComp
from lfd_pose_package.lib.helper_class import PoseHelpers

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rclpy.duration import Duration
# Stores known frames and offers frame graph requests

from lfd_pose_package.robot_navigator import BasicNavigator, NavigationResult

from geometry_msgs.msg import PoseStamped
import datetime
class CreateSMCPose(Node):
    '''
    Action class that decides when to create a waypoint
    '''
    def __init__(self):
        super().__init__('smc_pose_generator')
        self.initial_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.navigator = BasicNavigator()
        self.pose_helpers = PoseHelpers(self)
       
        self.timer_period = 1.0
        # Init laserscan subscription 
        self.lfd_comp_pose= self.create_subscription(LfdComp, '/comp_pose', self.cb_comp_subs, qos_profile= 10, callback_group= ReentrantCallbackGroup())
        
        
       
        #smc service client
        self.cli_smcsrv = self.create_client(SMCRes, '/smc_srv', callback_group= ReentrantCallbackGroup())
        while not self.cli_smcsrv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service SMC not available, waiting again...')
        self.req = SMCRes.Request()     
        
        self.srv_timer= self.create_timer(self.timer_period, self.cb_request_srv, callback_group= ReentrantCallbackGroup())
    
    def navigator_gotopose(self):
        self.navigator.goToPose(self.goal_pose)

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
                    self.navigator.cancelNavigation()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    self.goal_pose.pose.position.x = -3.0
                    self.navigator.goToPose(self.goal_pose)

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!') 


    def send_request(self):
        now = datetime.datetime.now()
        self.get_logger().info("[Request SMC service]: " + str(now.day)+"," + str(now.hour)+ ":" + str(now.minute)+ " - Trying to provide lfd service...")
        
        self.req.pose.data = [self.current_x, self.current_y, self.current_z]
        #self.lfd_srv_futures.append(self.cli_lfdsrv.call_async(self.req))
        self.future = self.cli_smcsrv.call_async(self.req)
        #rclpy.spin_until_future_complete(self, self.future)
        while not self.future.done(): pass
        return self.future.result()
    
    def cb_request_srv(self):
        response = self.send_request()
        self.get_logger().info('Client send request returned '+ str(response))
        if (response):
            self.get_logger().info('SMC Client send request returned')

            self.next_pose = response.nextpose.data
            self.next_pose[0] = self.next_pose[0]
            self.next_pose[1] = self.next_pose[1]
            self.next_pose[2] = self.next_pose[2]
            self.next_quat_x, self.next_quat_y, self.next_quat_z, self.next_quat_w = self.pose_helpers.rot2quatarnion(current_pose=current_pose_2d, next_pose=self.next_pose)
                
            self.goal_pose.header.frame_id = 'map'        
            self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()      
            self.goal_pose.pose.position.x = self.next_pose[0]
            self.goal_pose.pose.position.y = self.next_pose[1]
            self.goal_pose.pose.position.z = self.next_pose[2]
            self.goal_pose.pose.orientation.x = self.next_quat_x
            self.goal_pose.pose.orientation.y = self.next_quat_y
            self.goal_pose.pose.orientation.z = self.next_quat_z
            self.goal_pose.pose.orientation.w = self.next_quat_w

            #set initial pose for navigator
            self.navigator.setInitialPose(self.initial_pose)
            #wait for navigation to be active 
            self.navigator.waitUntilNav2Active()
            #check path 
            path = self.navigator.getPath(self.initial_pose, self.goal_pose)
        if (path!=None):
            self.get_logger().info('Path is valid')
            self.navigator_gotopose()
    
    
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
        self.get_logger().info("Initial_pose_created: " + 'x: ' + str(self.current_x)+" "+"y: " + str(self.current_y)+ " "+"z: " + str(self.current_z))


        self.goal_pose.header.frame_id = 'map'        
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()      
        self.goal_pose.pose.position.x = compmsg.nextpose.position.x
        self.goal_pose.pose.position.y = compmsg.nextpose.position.y
        self.goal_pose.pose.position.z = compmsg.nextpose.position.z
        self.goal_pose.pose.orientation.x = compmsg.nextpose.orientation.x
        self.goal_pose.pose.orientation.y = compmsg.nextpose.orientation.y
        self.goal_pose.pose.orientation.z = compmsg.nextpose.orientation.z
        self.goal_pose.pose.orientation.w = compmsg.nextpose.orientation.z

def main(args=None):
    logger = rclpy.logging.get_logger('smc_pose_generator')
    rclpy.init(args=args)
    """ node = CreateLfdPose()
    rclpy.spin(node)
    rclpy.shutdown()  """
    try:
        create_lfd = CreateSMCPose()
        executor = MultiThreadedExecutor(num_threads=7)
        executor.add_node(create_lfd)
        executor.spin()
       
    except Exception as e:
        logger.fatal('[smc_pose_generator]: Lfd generator node died')
        e = traceback.format_exc()
        logger.fatal(e)

    finally:
        executor.shutdown()
        create_lfd.destroy_node()
        rclpy.shutdown() 
  
if __name__ == '__main__':
  main()