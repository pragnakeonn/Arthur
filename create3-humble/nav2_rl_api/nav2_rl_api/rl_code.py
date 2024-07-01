#! /usr/bin/env python3

import copy
import os
import sys
import time
import numpy as np
import torch
from torch import nn as nn
from nav2_rl_api.Select_frontier import RLSF
import cv2
from rclpy.node import Node

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model_path = '/root/colcon_ws/src/nav2_rl_api/nav2_rl_api/model/rl_latest.pth'

class DrlAgent(Node):
    def __init__(self):
        super().__init__('rl_inference')
        
        self.model_path = model_path
        if (not self.model_path):
            return 
        self.model = RLSF()
        self.model.load_state_dict(torch.load(model_path),strict=False)
    def forward(self, mapData, curr_pos):
        frontiers, img = self.getfrontier(mapData, curr_pos)
        self.get_logger().info('Frontier shape...'+str(frontiers.shape))
      
        #torch.zeros()
        img = img/255.0
        img = torch.from_numpy(img).to(torch.float32)
        img = img.permute(2,0,1).unsqueeze(0)
        self.get_logger().info('img shape...'+str(img.shape))
        self.get_logger().info('curr pos...'+str(curr_pos))
        poses =[]
        for f in frontiers:
            poses.append(curr_pos[:-1])
        self.get_logger().info('poses...'+str(np.array(poses).shape))
        frontiers = torch.from_numpy(np.array(frontiers)).to(torch.float32)
        poses = torch.from_numpy(np.array(poses)).to(torch.float32)
        best_frontier = self.model.get_action(frontiers,img,poses)
        best_frontier = best_frontier.tolist()
        self.get_logger().info('frontiers indexed...'+str(best_frontier))
        return best_frontier,img
    def getfrontier(self,mapData, curr_pos, alpha =1):
        data=mapData.data
        w=mapData.info.width
        h=mapData.info.height
        resolution=mapData.info.resolution
        Xstartx=mapData.info.origin.position.x
        Xstarty=mapData.info.origin.position.y
        data=mapData.data
        w=mapData.info.width
        h=mapData.info.height
        resolution=mapData.info.resolution
        Xstartx=mapData.info.origin.position.x
        Xstarty=mapData.info.origin.position.y
	 
        img = np.zeros((h, w, 1), np.uint8)
	
        for i in range(0,h):
            for j in range(0,w):
                    if data[i*w+j]==100:
                        img[i,j]=0
                    elif data[i*w+j]==0:
                        img[i,j]=255
                    elif data[i*w+j]==-1:
                         img[i,j]=205
        o=cv2.inRange(img,0,1)
        edges = cv2.Canny(img,0,255)
        contours, hierarchy = cv2.findContours(o,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(o, contours, -1, (255,255,255), 5)
        o=cv2.bitwise_not(o) 
        res = cv2.bitwise_and(o,edges)
	#------------------------------
        frontier=res
        contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frontier, contours, -1, (255,255,255), 2)
        contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        all_pts=[]
        if len(contours)>0:
            upto=len(contours)-1
            i=0
            maxx=0
            maxind=0
        for i in range(0,len(contours)):
            cnt = contours[i]
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            xr=cx*resolution+Xstartx
            yr=cy*resolution+Xstarty
            pt=[np.array([xr,yr])]
            if len(all_pts)>0:
                all_pts=np.vstack([all_pts,pt])
            else:
                all_pts=pt
        return all_pts, img
    
         


