#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import PointCloud2
from lfd_interfaces.msg import LfdPose
import os
import math
import numpy as np
#import xml.etree.ElementTree as ET
#
import requests
import psutil as ps
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup



class PoseHelpers():

    def __init__(self, node):
        self.node = node
    def rot2quatarnion(self, current_pose, next_pose):
        self.pt_1=current_pose
        self.pt_2=next_pose
        a_vec = np.array(self.pt_1)/np.linalg.norm(np.array(self.pt_1))
        b_vec = np.array(self.pt_2)/np.linalg.norm(np.array(self.pt_2))

        cross = np.cross(a_vec, b_vec)
        ab_angle = np.arccos(np.dot(a_vec,b_vec))


        vx = np.array([[0,-cross[2],cross[1]],[cross[2],0,-cross[0]],[-cross[1],cross[0],0]])
        R = np.identity(3)*np.cos(ab_angle) + (1-np.cos(ab_angle))*np.outer(cross,cross) + np.sin(ab_angle)*vx


        validation=np.matmul(R,a_vec)
        print(R)
        if (R[0][0]+R[1][1]+R[2][2] > 0):
            q1=(1/2)*np.sqrt(1+R[0][0]+R[1][1]+R[2][2])
        else:
            q1 = (1/2)*np.sqrt((np.square(R[2][1]-R[1][2])+np.square(R[0][2]-R[2][0])+np.square(R[1][0]-R[0][1]))/(3-R[0][0] - R[1][1]- R[2][2]))
        if (R[0][0]-R[1][1]-R[2][2] > 0):
            q2=(1/2)*np.sqrt(1+R[0][0]-R[1][1]-R[2][2])
        else: 
            q2 = (1/2)*np.sqrt((np.square(R[2][1]-R[1][2])+np.square(R[0][2]+R[2][0])+np.square(R[2][0]-R[0][2]))/(3-R[0][0]+R[1][1]+R[2][2]))
        if (-R[0][0]+R[1][1]-R[2][2] > 0):
            q3=(1/2)*np.sqrt(1-R[0][0]+R[1][1]-R[2][2])
        else:
            q3 = (1/2)*np.sqrt((np.square(R[0][2]-R[2][0])+np.square(R[0][1]+R[1][0])+np.square(R[1][2]+R[2][1]))/(3+R[0][0]+R[1][1]+R[2][2]))
        if (-R[0][0]-R[1][1]+R[2][2] > 0):
            q4=(1/2)*np.sqrt(1-R[0][0]-R[1][1]+R[2][2])
        else:
            q4=(1/2)*np.sqrt((np.square(R[1][0]-R[0][1])+np.square(R[2][0]+R[0][2])+np.square(R[2][1]+R[1][2]))/(3+R[0][0]+R[1][1]-R[2][2]))

        return q1, q2, q3, q4
    def euler_from_quaternion(self, x, y, z, w):
        """Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        self.roll_x = math.atan2(t0, t1)
      
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        self.pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        self.yaw_z = math.atan2(t3, t4)
        return self.roll_x, self.pitch_y, self.yaw_z
    def get_direction(self, laser_scan):
        range = laser_scan.ranges
        angle_min = laser_scan.angle_min
        increment = laser_scan.angle_increment
        
        print("increment inside get direction:", angle_min, increment)
        #discard infinity in ranges
        #filtered_range = [v for v in range if not (math.isinf(v) or math.isnan(v))]
        #self.get_logger().info('np.isinfinite range ' + str(np.isfinite(range)))
        print(np.isfinite(range))
        non_finite = [i for i,v in enumerate(range) if (math.isinf(v) or math.isnan(v))]
        for f in non_finite:
            range[f] = 0
        #range[~np.isfinite(range)] = 0.0
        # angles = [angle_min+index*increment for index in range(len(range))]
        index = np.argmax(range)
        #print("index inside get direction:", index)
        self.angle = angle_min + index*increment
        print("Angle inside get direction:", self.angle)
        #self.get_logger().info('Get direction return angle value ' + str(self.angle))
        return self.angle, range[index]
    def get_euclidean_distance(self, curr_pose, next_pose):
        a = np.array((curr_pose[0], curr_pose[1], curr_pose[2]))
        b = np.array((next_pose[0], next_pose[1], next_pose[2]))
        dist = np.linalg.norm(a-b)
        return dist
        
    