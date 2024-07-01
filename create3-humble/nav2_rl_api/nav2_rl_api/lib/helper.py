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