#!/usr/bin/env python3
import torch
import torch.nn as nn
import torch.utils
import torch.utils.data
from torchvision import datasets, transforms
import rclpy
from sensor_msgs.msg import PointCloud2
from lfd_interfaces.msg import LfdPose
import os
import numpy as np
from .model_lstm import LSTMRegressor
#import xml.etree.ElementTree as ET
from sensor_msgs.msg import LaserScan
import requests
import psutil as ps
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
x_dim = 12
h_dim = 32
z_dim = 12
n_layers =  2
n_epochs = 100
clip = 10
learning_rate = 1e-4
batch_size = 32
ranseed = 128
print_every = 10
save_every = 10

#trajectory_Obj = Trajectory("/home/suman/person_trajectory/varRNN/VariationalRNN/VariationalRecurrentNeuralNetwork-master/data/thor")

#init model + optimizer + datasets
#train_loader = torch.utils.data.DataLoader(trajectory_Obj, batch_size=batch_size, shuffle=True)

#val_loader = torch.utils.data.DataLoader(trajectory_Obj, batch_size=16, shuffle=False)

model = LSTMRegressor(embedding_dim=128,hidden_dim=128,output_size=3)
#/root/catkin_ws/src/lfd_services/lfd_services/lib/models/lstm_state_dict_31.pth
state_dict = torch.load('/root/colcon_ws/src/lfd_services/lfd_services/models/lstm_trial5_state_dict_91.pth')
#model = model.cuda()
model.load_state_dict(state_dict)
#model = model.cuda()
model = model.to(device) # Set model to gpu
model.eval() 
start_pos = np.array([0.0, 0.0,0.0])


class LfdMsg(): #in order to avoid filling the 3 values in every instance in 3 different lines
    def __init__(self, name, state, msg_info):
        self.diag_mess = LfdPose()
        self.diag_mess.name = name
        self.diag_mess.state = state
        self.diag_mess.info_msg = msg_info
    
    def msg(self):
        return self.diag_mess

class LfdHelpers():

    def __init__(self, node):
        self.node = node
        #self.sub_node=sub_node
        #self.navigate_node=navigate_node
        #self.pose_flag = self.create_pose()
        
    def create_pose(self,current_pose, angle ):
        print("[create_pose_helper]: Called this helper with section " + str(angle))  
        degree=torch.tensor(angle)
        pose=torch.tensor(current_pose)
        print("pose shape", pose.shape)
        print("degree shape", degree.shape)
        print("pose shape", pose)
        print("degree shape", degree)
        #self.get_logger().info('Want to get next pose from lfd')
        pos = model(pose, degree)
        #np_pose= pos.numpy()
        next_pose = pos.numpy()
        return next_pose
    