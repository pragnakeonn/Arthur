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


from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import AssistedTeleop, BackUp, Spin
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose
from nav2_msgs.action import SmoothPath
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from irobot_create_msgs.action import Dock, RotateAngle


class RotationResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

class BasicNavigator(Node):

    def __init__(self, node_name='basic_rotator', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        self.rotate_action_client = ActionClient(self, RotateAngle, '/rotate_angle')

    def send_goal(self, angle, max_rotation_speed=0.5):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = max_rotation_speed
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.info("'NavigateToPose' action server not available, waiting...")
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))
