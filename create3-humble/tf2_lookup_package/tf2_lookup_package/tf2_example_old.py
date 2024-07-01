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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import asyncio
import traceback
from rclpy.executors import MultiThreadedExecutor
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
        
        self.current_x = 0.0
        self.current_y = 0.0 
        self.current_yaw = 0.0
        cg = ReentrantCallbackGroup()
        timer_period = 0.33
        self.timer = self.create_timer(timer_period, self.on_timer)
    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = 'map'
        self.get_logger().info('Buffer!'+str(self.tf_buffer))
        tf_future = self.tf_buffer.wait_for_transform_async(
                        target_frame=to_frame_rel,
                        source_frame=from_frame_rel,
                        time=rclpy.time.Time()
                    )

        rclpy.spin_until_future_complete(self, tf_future)
        self.get_logger().info('Got it!')
    #   print("Got it!")

#                world_to_reference_transform = self.tf_buffer.lookup_transform(
#                    pose_stamped.header.frame_id,
#                    reference_frame,
#                    rclpy.time.Time(),
#                    timeout=rclpy.duration.Duration(seconds=5.0))

        world_to_reference_transform = asyncio.run(self.tf_buffer.lookup_transform_async(
                        to_frame_rel,
                        from_frame_rel,
                        rclpy.time.Time()
                    ))
        self.get_logger().info('World to reference transform done')
        self.current_x = world_to_reference_transform.transform.translation.x
        self.current_y = world_to_reference_transform.transform.translation.y    
        roll, pitch, yaw = self.tf_helpers.euler_from_quaternion(
        world_to_reference_transform.transform.rotation.x,
        world_to_reference_transform.transform.rotation.y,
        world_to_reference_transform.transform.rotation.z,
        world_to_reference_transform.transform.rotation.w)      
        self.current_yaw = yaw    
        self.get_logger().info('current_x'+str(self.current_x))
        msg = Float64MultiArray()
        msg.data = [self.current_x, self.current_y, self.current_yaw]   
        self.publisher_2d_pose.publish(msg) 
        self.get_logger().info('Publisher called'+str(msg.data))
  
 
def main(args=None):
    logger = rclpy.logging.get_logger('tf2_lookup_package')
  # Initialize the rclpy library
    rclpy.init(args=args)
  
    # Create the node
    frame_listener_node = FrameListener()
  
  
    rclpy.spin(frame_listener_node)
  
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    """ try:
        frame_listener_node = FrameListener()
        executor = MultiThreadedExecutor()
        executor.add_node(frame_listener_node)
        executor.spin()
       
    except Exception as e:
        logger.fatal('[Tf2_lookup]: Rf2_lookup node died')
        e = traceback.format_exc()
        logger.fatal(e)

    finally:
        executor.shutdown()
        frame_listener_node.destroy_node()
        rclpy.shutdown() """