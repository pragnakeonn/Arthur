#!/usr/bin/env python3 
 
"""
Description:
Publish the coordinate transformation between the map frame
and the base_link frame.
The output is [x,y,yaw]. yaw is -pi to pi
-------
Subscription Topics:
/tf - geometry_msgs/TransformStamped[]
-------
Publishing Topics:
/map_to_base_link_pose2d – std_msgs/Float64MultiArray
-------
Author: Addison Sears-Collins
Website: AutomaticAddison.com
Date: November 25, 2021
"""
 
# Import the ROS client library for Python 
import rclpy 
 
# Enables the use of rclpy's Node class
from rclpy.node import Node 
from rclpy.duration import Duration
 
# Base class to handle exceptions
from tf2_ros import TransformException 
 
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
 
# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener 
 
# Handle float64 arrays
from std_msgs.msg import Float64MultiArray 
from lfd_interfaces.msg import LfdPose
 
# Math library
import math 
 
 
class FrameListener(Node):
    """
    Subclass of the Node class.
    The class listens to coordinate transformations and 
    publishes the 2D pose at a specific time interval.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
    
    # Initiate the Node class's constructor and give it a name
        super().__init__('map_base_link_frame_listener')
 
    # Declare and acquire `target_frame` parameter
        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter(
        'target_frame').get_parameter_value().string_value
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
      
        # Create publisher(s)  
        
        # This node publishes the 2d pose.
        # Maximum queue size of 1. 
        self.publisher_2d_pose = self.create_publisher(
        LfdPose, 
        '/map_to_base_link_pose2d', 
        1)
    
        # Call on_timer function on a set interval
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.on_timer)
     
        # Current position and orientation of the target frame with respect to the 
        # reference frame. x and y are in meters, and yaw is in radians.
        self.current_x = 0.0
        self.current_y = 0.0 
        self.current_yaw = 0.0
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians 
    def on_timer(self):
        """
        Callback function.
        This function gets called at the specific time interval.
        """
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'map'
    
        trans = None
        #self.get_logger().info('Timer!')
        try:
            now = rclpy.time.Time()
            
            trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now,rclpy.duration.Duration(seconds=0.3) )
            #self.get_logger().info('Trans got!'+str(trans))
        except TransformException as ex:
            self.get_logger().info(
            f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        self.current_x = trans.transform.translation.x
        self.current_y = trans.transform.translation.y 
        self.current_z = trans.transform.translation.z 
        #roll, pitch, yaw = self.euler_from_quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)      
        #self.current_yaw = yaw   
        
        quat_x= trans.transform.rotation.x 
        quat_y= trans.transform.rotation.y
        quat_z= trans.transform.rotation.z
        quat_w= trans.transform.rotation.w
        curr_pose = LfdPose()
        #self.get_logger().info('Creating curr_pose')
        curr_pose.header.frame_id = 'map'
        
        curr_pose.header.stamp = self.get_clock().now().to_msg()
        #rclpy.time.Time()
        #self.get_logger().info('Calling publisher 3')
        curr_pose.pose.position.x = self.current_x
        curr_pose.pose.position.y = self.current_y
        curr_pose.pose.position.z = self.current_z
        curr_pose.pose.orientation.x = quat_x
        curr_pose.pose.orientation.y = quat_y
        curr_pose.pose.orientation.z = quat_z
        curr_pose.pose.orientation.w = quat_w
        self.publisher_2d_pose.publish(curr_pose) 

        

def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    
    frame_listener_node = FrameListener()
    rclpy.spin(frame_listener_node)
    rclpy.shutdown()
  
  
  
if __name__ == '__main__':
  main()