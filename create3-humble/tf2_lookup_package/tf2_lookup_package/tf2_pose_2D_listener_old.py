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
 
# Math library
import math 
from tf2_lookup_package.lib.tf2_helper import TfHelpers
 
 
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
    self.tf_helpers = TfHelpers(self)  
    # Create publisher(s)  
      
    # This node publishes the 2d pose.
    # Maximum queue size of 1. 
    self.publisher_2d_pose = self.create_publisher(
      Float64MultiArray, 
      '/map_to_base_link_pose2d', 
      1)
 
    # Call on_timer function on a set interval
    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.on_timer)
    #self.on_timer()
    # Current position and orientation of the target frame with respect to the 
    # reference frame. x and y are in meters, and yaw is in radians.
    self.current_x = 0.0
    self.current_y = 0.0 
    self.current_yaw = 0.0
     
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
     
    try:
      now = rclpy.time.Time()
      #print(self.get_clock().now())
      #now = self.get_clock().now()
      #now = self.get_clock().now() # - Duration(seconds=0.05)
      #print("trying section")
      #trans = self.tf_buffer.lookup_transform(
      #            to_frame_rel,
      #            from_frame_rel,
      #            now,rclpy.duration.Duration(seconds=0.3) )
      trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,now,Duration(seconds=0.05) )
      #print(trans)
    except TransformException as ex:
      self.get_logger().info(
        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
      return
       
    # Publish the 2D pose
    self.current_x = trans.transform.translation.x
    self.current_y = trans.transform.translation.y    
    roll, pitch, yaw = self.tf_helpers.euler_from_quaternion(
      trans.transform.rotation.x,
      trans.transform.rotation.y,
      trans.transform.rotation.z,
      trans.transform.rotation.w)      
    self.current_yaw = yaw    
    msg = Float64MultiArray()
    msg.data = [self.current_x, self.current_y, self.current_yaw]   
    self.publisher_2d_pose.publish(msg) 
   
  
 
def main(args=None):
  logger = rclpy.logging.get_logger('tf2_lookup_package')
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  frame_listener_node = FrameListener()
  
  
  rclpy.spin(frame_listener_node)
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
    
    
  
if __name__ == '__main__':
  main()