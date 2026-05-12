import numpy as np
import torch 
from torch import nn

class CopycatNetwork(nn.Module):

    def __init__(self, pred_length):
        super().__init__()
        self.pred_length = pred_length
        self.device='cpu' #Init device, use to function to change device

        #Recurrent layer
        self.rnn = nn.LSTMCell(2,50)
        #Init hidden and cell output 
        self.hidden = nn.Parameter(torch.zeros([50]))
        self.cell = nn.Parameter(torch.zeros([50]))

        self.ctrl = nn.Sequential(nn.Linear(50, 50), 
                        nn.Linear(50, 2+2*pred_length), 
                        nn.Tanh())
        
        self.waypoints = nn.Sequential(nn.Linear(50, 50), 
                        nn.Linear(50, 2*pred_length))


		
    def reset_RNN(self):
        self.hidden = nn.Parameter(torch.zeros(50).to(self.device))
        self.cell = nn.Parameter(torch.zeros(50).to(self.device))

    def to(self, device):
        super().to(device)
        self.device = device
        
    def forward(self, input):
        outputs=dict()
        
        x,_ = self.rnn(input,(self.hidden, self.cell))
        pred_ctrl = self.ctrl(x)
        outputs["acker_speed_branches"]=pred_ctrl[0]
        outputs["acker_steer_branches"]=pred_ctrl[1]
        outputs["future_acker_speed"]=pred_ctrl[2:2+self.pred_length]
        outputs["future_acker_steer"]=pred_ctrl[2+self.pred_length:]

        pred_waypoints = self.waypoints(x)
        outputs["pred_wp"]=pred_waypoints

        return outputs
