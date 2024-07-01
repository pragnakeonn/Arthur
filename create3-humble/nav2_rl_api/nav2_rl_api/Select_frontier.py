#! /usr/bin/env python3


import torch
import torch.nn as nn
import numpy as np
import torch.nn.functional as F

class RLSF(nn.Module):
    def __init__(self, ht=1, wd=1):
        super().__init__()
        self.conv1 = nn.Conv2d(1,8,4,1)
        self.conv2 = nn.Conv2d(8,8,4,1)
        self.linear = nn.Linear(4,1)
        self.relu = nn.ReLU()
    def get_action(self,frontiers, img,pose):
        try:
            
            x = self.relu(self.conv1(img))
            pose_front = torch.cat((pose, frontiers))
            y = self.linear(pose_front)
            activation = F.softmax(y)
            dist = []
            for i in range(len(frontiers)):
                x=frontiers[i]
                dist.append(np.sqrt((x[0] - pose[0])**2 + (x[1] - pose[1])**2))
            idx = np.flip(np.argsort(dist))
        except:
            self.__init__()
            x = self.relu(self.conv1(img))
            pose_front = torch.cat((pose, frontiers), dim=1)
            y = self.linear(pose_front)
            activation = F.softmax(y)
            dist = []
            fronts = frontiers.numpy()
            pos = pose[0].numpy()
            for i in range(fronts.shape[0]):
                x=frontiers[i]
                dist.append(np.sqrt((x[0] - pos[0])**2 + (x[1] - pos[1])**2))
            idx = np.flip(np.argsort(dist))


        return fronts[idx]

        