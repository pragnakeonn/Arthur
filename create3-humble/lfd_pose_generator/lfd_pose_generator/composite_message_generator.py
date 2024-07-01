#!/usr/bin/env python3
import rclpy
import traceback
from rclpy.node import Node
from lfd_interfaces.msg import LfdPose, LfdComp
from sensor_msgs.msg import LaserScan
from lfd_pose_package.lib.helper_class import PoseHelpers
class CreateLfdModelInput(Node):
    '''
    Action class that decides when to create a waypoint
    '''
    def __init__(self):
        super().__init__('lfd_model_input')
        self.current_poses = []
        self.angles = []
        self.cur_index = 0
        self.timer_period = 1.0
        self.pose_helpers = PoseHelpers(self)
        self.lasersub= self.create_subscription(LaserScan,
          '/scan', self.cb_laserscan_subs, qos_profile= 10)
        
        self.transform= self.create_subscription(LfdPose,
            '/map_to_base_link_pose2d', self.cb_pose2d_subs, qos_profile= 10)
        self.get_logger().info('LaserScan and pose subscription created')

        self.publisher_composite_msg = self.create_publisher(LfdComp, 
        '/model_input_msg', 1)

        self.timer= self.create_timer(self.timer_period, self.cb_create_composite)

    def cb_laserscan_subs(self, msg):
        self.get_logger().info('I heard angle "%f"' %msg.angle_min )
        self.angles.append(self.pose_helpers.get_direction(laser_scan=msg))
        self.get_logger().info('inside scan listener callback,angle value ' + str(self.angles[-1]))

    def cb_pose2d_subs(self, msg):
        self.get_logger().info('Pose subscriber cb')
        x_pt= msg.pose.position.x 
        y_pt= msg.pose.position.y 
        z_pt= msg.pose.position.z 
        quat_x= msg.pose.orientation.x 
        quat_y = msg.pose.orientation.y 
        quat_z = msg.pose.orientation.z 
        quat_w = msg.pose.orientation.w 
       
        #self.get_logger().info('inside scan listener callback,angle value ' + str(self.current_poses))
        #z_pt=0
        self.current_pose =[x_pt, y_pt, z_pt, quat_x, quat_y, quat_z,quat_w]
        self.current_poses.append(self.current_pose)
        #self.get_logger().info('inside pose 2D listener listener callback,angle value ' + str(self.current_poses))

    def cb_create_composite(self):
        if len(self.current_poses) >1 and len(self.angles) > 1:
            #self.get_logger().info('create_composite Cb, print pose...'+str(self.current_poses[0]))
            self.get_logger().info('create_composite Cb, print angle...'+str(self.angles))  
            self.angle = self.angles[self.cur_index] 
            current_pose_2d = self.current_poses[self.cur_index]
            comp_msg = LfdComp()
            #self.get_logger().info('Creating composite msg now') 
            comp_msg.header.frame_id = 'map'
            comp_msg.header.stamp = self.get_clock().now().to_msg()
            comp_msg.angle = self.angle
            comp_msg.pose.position.x = current_pose_2d[0]
            comp_msg.pose.position.y = current_pose_2d[1]
            comp_msg.pose.position.z = current_pose_2d[2]
            comp_msg.pose.orientation.x = current_pose_2d[3]
            comp_msg.pose.orientation.y = current_pose_2d[4]
            comp_msg.pose.orientation.z = current_pose_2d[5]
            comp_msg.pose.orientation.w = current_pose_2d[6]
            self.cur_index = self.cur_index+1
            self.get_logger().info('create_composite Cb, print index...'+str(self.cur_index))
            self.publisher_composite_msg.publish(comp_msg) 
            
def main(args=None):
    logger = rclpy.logging.get_logger('lfd_model_input')
    rclpy.init(args=args)
    node = CreateLfdModelInput()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
  main()