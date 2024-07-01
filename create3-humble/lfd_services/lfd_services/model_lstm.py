#!/usr/bin/env python3
import torch.nn as nn
from torch.nn.utils.rnn import pack_padded_sequence, pad_packed_sequence
import torch

class LSTMRegressor(nn.Module):

    def __init__(self, embedding_dim, hidden_dim,output_size):
        super(LSTMRegressor, self).__init__()
        
        self.embedding = nn.Linear(4,embedding_dim)
        
        # 2. LSTM Layer
        self.lstm = nn.LSTM(embedding_dim, hidden_dim, num_layers=1)
        
        # 3. Optional dropout layer
        self.dropout_layer = nn.Dropout(p=0.5)

        # 4. Dense Layer
        self.hidden2traj = nn.Linear(hidden_dim, output_size)
        
    def forward(self, pos, degree):

        embeddings = self.embedding(torch.cat((pos,degree.unsqueeze(2)),dim =2))
        print("[lfd_srv]:here")
        
        
        lstm_output, _ = self.lstm(embeddings)
        
        lstm_output = self.dropout_layer(lstm_output)
        
        traj_pos = self.hidden2traj(lstm_output)
        return traj_pos
    #def sample(self,pos, slope):
