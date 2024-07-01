#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import traceback
import base64
from rclpy.action import ActionClient
from subprocess import Popen, call, PIPE
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan
from lfd_interfaces.srv import LfdRes
#from lfd_interfaces.msg import Lfd
from lfd_services.lib.lfd_helpers import LfdHelpers
#from robin_interfaces.action import CommandManager
#from keonn_logger import keonn_file_logger
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from threading import Event
import threading
import time
import torch
import torch.nn as nn
import torch.utils
import torch.utils.data
from torchvision import datasets, transforms
import numpy as np
from .model_lstm import LSTMRegressor
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
state_dict = torch.load('/root/colcon_ws/src/lfd_services/lfd_services/models/lstm_trial5_state_dict_91.pth')
#model = model.cuda()
model.load_state_dict(state_dict)
#model = model.cuda()
model = model.to(device) # Set model to gpu
model.eval() 

class LfdServicesController(Node):    

    def __init__(self): 
        super().__init__('lfd_services')
        self.get_logger().debug("[lfd_services]: Starting services")
        cbb = ReentrantCallbackGroup()
        # Init Diagnostics Service
        self.lfd_helpers = LfdHelpers(self)

        #Init lfd service
        self.create_service(LfdRes, '/lfd_srv', self.cb_lfd_srv, callback_group= ReentrantCallbackGroup())
        
    def create_pose(self,current_pose, angle ):
        self.get_logger().info("[lfd_srv]: Called create_pose function " + str(angle))  
        degree=torch.tensor(angle).unsqueeze(0).unsqueeze(0).to(device)
        self.get_logger().info("[lfd_srv]: Called degree torch " + str(angle))
        pose=torch.tensor(current_pose).unsqueeze(0).unsqueeze(0).to(device)
        
        self.get_logger().info("[lfd_srv]: Called pose torch with dimension " + str(pose.shape))
        
        #self.get_logger().info('Want to get next pose from lfd')
        pos = model(pos=pose, degree=degree) #check this line
        self.get_logger().info("[lfd_srv]: Called model " + str(pos.shape))
        #np_pose= pos.numpy()
        next_pose = pos.cpu().detach().numpy()
        self.get_logger().info("[lfd_srv]: returning nextpose " + str(next_pose[0]))
        return next_pose[0][0]
       
       
    def cb_lfd_srv(self, req, res):
        '''
        Implements the lfd service while the robot is doing SLAM to get the next pose to move.
        :param req:
        :return:
        '''
        self.get_logger().info("[lfd_srv]: Called the service with section " + str(req.angle))  
        try:
            self.get_logger().info("[lfd_srv]: Inside Try section " + str(req.angle)) 
            #res.nextpose.data = req.pose.data*3
            pose= self.create_pose(current_pose=req.pose.data, angle=req.angle)
            self.get_logger().info("[lfd_srv]: Got the pose " + str(pose))
            temp = [float(pose[0]), float(pose[1]), float(pose[2])]
            self.get_logger().info("[lfd_srv]:" + str(temp))
            res.nextpose.data = temp
            self.get_logger().info("[lfd_srv]: set next pose ")
            res.success = True
        except Exception as e:
            self.get_logger().error("[lfd_srv]: Error getting next pose, probably not found in the path") 
            self.get_logger().error(str(e)) 
            res.success = False
            res.nextpose.data = [0,0,0]
        return res
        


        
def main(args=None):
    logger = rclpy.logging.get_logger('lfd_services')
    rclpy.init(args=args)
    """ lfd_serv = LfdServicesController()

    rclpy.spin(lfd_serv)

    rclpy.shutdown() """

    try:
        lfd_serv = LfdServicesController()
        executor = MultiThreadedExecutor()
        executor.add_node(lfd_serv)
        executor.spin()
       
    except Exception as e:
        logger.fatal('[lfd_services]: Lfd services node died')
        e = traceback.format_exc()
        logger.fatal(e)

    finally:
        executor.shutdown()
        lfd_serv.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()
