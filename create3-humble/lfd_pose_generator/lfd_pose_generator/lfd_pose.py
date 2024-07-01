#!/usr/bin/env python3
import rclpy
import traceback
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from lfd_interfaces.action import Navigate
from lfd_interfaces.srv import LfdRes
from lfd_interfaces.msg import LfdPose, LfdComp
from lfd_pose_generator.lib.helper_class import PoseHelpers
from std_msgs.msg import Float64MultiArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from tf2_ros import TransformException 
from rclpy.duration import Duration
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
#from robot_navigator import BasicNavigator, NavigationResult
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from irobot_create_msgs.action import NavigateToPosition
import datetime


class CreateLfdPose(Node):
    '''
    Action class that decides when to create a waypoint
    '''
    def __init__(self):
        super().__init__('lfd_pose_generator')
        self.pose_helpers = PoseHelpers(self)
        #self.cg = None
        self.lfd_srv_futures = []
        self.incomplete_futures = []
        self.current_poses = []
        self.angles = []
        self.cur_index = 0
        #self.cg = ReentrantCallbackGroup()
        #cg = MutuallyExclusiveCallbackGroup()
        # Init laserscan subscription 
        self.wp_completed_msg = None
        self.next_pose = list()
        self.tf_buffer = Buffer()
        self.timer_period = 1.0
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lasersub= self.create_subscription(LfdComp, 
        '/model_input_msg', self.cb_modelinput_subs, qos_profile= 10, callback_group= ReentrantCallbackGroup())
        
        
        self.get_logger().info('Model Input subscription created')
        #Init lookup parameter for transform
        #self.declare_parameter('target_frame', 'base_link')
        #self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
 
        

        # Call on_timer function on a set interval
        
        #self.pose_timer= self.create_timer(self.timer_period, self.transform_timer, callback_group=self.cg)
        #self.get_logger().info('Transform Callback Timer created')
        # Current position and orientation of the target frame with respect to the 
        # reference frame. x and y are in meters, and yaw is in radians.
        self.angle = 0.0
        self.current_x = 0.0
        self.current_y = 0.0 
        self.current_z = 0.0
        
        self.cli_lfdsrv = self.create_client(LfdRes, '/lfd_srv', callback_group= ReentrantCallbackGroup())
        while not self.cli_lfdsrv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LfdRes.Request()     
        
        self.srv_timer= self.create_timer(self.timer_period, self.cb_request_srv, callback_group= ReentrantCallbackGroup())
        #self.get_logger().info('Client timer created')
        self.robot_status_publisher = self.create_publisher(PoseStamped, '/goal_pose', 5, callback_group= ReentrantCallbackGroup())
        #self.timer_4= self.create_timer(self.timer_period, self.pose_pub, callback_group=self.cg)
        
        self.get_logger().info('Publisher timer created')
        
        #self.action_timer = self.create_timer(self.timer_period, self.cb_navigate_client, callback_group= ReentrantCallbackGroup())
    def lfd_pose_pub(self):
        #goal_poses = []
        #now = datetime.datetime.now()
        ct = datetime.datetime.now()
        ts = ct.timestamp()
        self.get_logger().info('rot2quatarnion, quat_x' + str(self.next_quat_x))
        self.get_logger().info('rot2quatarnion, quat_y' + str(self.next_quat_y))
        self.get_logger().info('rot2quatarnion, quat_z' + str(self.next_quat_z))
        self.get_logger().info('rot2quatarnion, quat_w' + str(self.next_quat_w))
        goal_pose = PoseStamped()
        #goal_pose = create3NavPose()
        self.get_logger().info('Calling publisher 1')
        goal_pose.header.frame_id = 'map'
        self.get_logger().info('Calling publisher 2')
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        #rclpy.time.Time()
        self.get_logger().info('Calling publisher 3')
        goal_pose.pose.position.x = self.next_pose[0]
        goal_pose.pose.position.y = self.next_pose[1]
        goal_pose.pose.position.z = self.next_pose[2]
        goal_pose.pose.orientation.x = self.next_quat_x
        goal_pose.pose.orientation.y = self.next_quat_y
        goal_pose.pose.orientation.z = self.next_quat_z
        goal_pose.pose.orientation.w = self.next_quat_w
        self.get_logger().info('Calling publisher 4')
        self.robot_status_publisher.publish(goal_pose)
        self.get_logger().info('Publisher called 4')
    def send_request(self):
        now = datetime.datetime.now()
        self.get_logger().info("[Request Lfd service]: " + str(now.day)+"," + str(now.hour)+ ":" + str(now.minute)+ " - Trying to provide lfd service...")
        self.req.angle = self.angle
        self.req.pose.data = [self.current_x, self.current_y, self.current_z]
        #self.lfd_srv_futures.append(self.cli_lfdsrv.call_async(self.req))
        self.future = self.cli_lfdsrv.call_async(self.req)
        #rclpy.spin_until_future_complete(self, self.future)
        while not self.future.done(): pass
        return self.future.result()
        
        #self.print_periodic_upload_info(future.result())
    def cb_modelinput_subs(self, msg):
        self.get_logger().info('Composite Message is received')
        self.angle = msg.angle
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        self.quat_x = msg.pose.orientation.x
        self.quat_y = msg.pose.orientation.y
        self.quat_z = msg.pose.orientation.z
        self.quat_w = msg.pose.orientation.w
    def cb_request_srv(self):
        current_pose_2d = [self.current_x, self.current_y, self.current_z]
        response = self.send_request()
        self.get_logger().info('Client send request returned '+ str(response))
        if (response):
                self.get_logger().info('Client send request returned')

                self.next_pose = response.nextpose.data
                self.get_logger().info('Request Received, next_pose' + str(self.next_pose[0]))
                self.get_logger().info('Request Received, next_pose' + str(self.next_pose[1]))
                self.get_logger().info('Request Received, next_pose' + str(self.next_pose[2]))        
                self.cur_index =  self.cur_index + 1
                    #current_pose = [self.current_x, self.current_y, self.current_yaw]
                self.get_logger().info('Publishing message is getting prepared, current_pose' + str(current_pose_2d[0]))
                self.get_logger().info('Publishing message is getting prepared, current_pose' + str(current_pose_2d[1]))
                self.get_logger().info('Publishing message is getting prepared, current_pose' + str(current_pose_2d[2]))
                self.get_logger().info('Publishing message is getting prepared, next_pose' + str(self.next_pose[0]))
                self.get_logger().info('Publishing message is getting prepared, next_pose' + str(self.next_pose[1]))
                self.get_logger().info('Publishing message is getting prepared, next_pose' + str(self.next_pose[2]))
                
                self.next_quat_x, self.next_quat_y, self.next_quat_z, self.next_quat_w = self.pose_helpers.rot2quatarnion(current_pose=current_pose_2d, next_pose=self.next_pose)
                """ self.next_quat_x = 0.0
                self.next_quat_y = 0.0
                self.next_quat_z = 0.0
                self.next_quat_w = 1.0
 """

                """ navigator = BasicNavigator()
                initial_pose = PoseStamped()
                initial_pose.header.frame_id = 'map'
                initial_pose.header.stamp = navigator.get_clock().now().to_msg()
                initial_pose.pose.position.x = self.current_x
                initial_pose.pose.position.y = self.current_y
                initial_pose.pose.position.z = self.current_z
                initial_pose.pose.orientation.x = self.quat_x
                initial_pose.pose.orientation.y = self.quat_y
                initial_pose.pose.orientation.z = self.quat_z
                initial_pose.pose.orientation.w = self.quat_w
                navigator.setInitialPose(initial_pose) """
                
                #pos_x, pos_y, pos_z, r, p, y, w
                #self.cb_navigate_client(quat_x=quat_x, quat_y=quat_y, quat_z=quat_z, quat_w=quat_w)
                self.lfd_pose_pub()

    
def main(args=None):
    logger = rclpy.logging.get_logger('lfd_pose_generator')
    rclpy.init(args=args)
    """ node = CreateLfdPose()
    rclpy.spin(node)
    rclpy.shutdown()  """
    try:
        create_lfd = CreateLfdPose()
        executor = MultiThreadedExecutor(num_threads=7)
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