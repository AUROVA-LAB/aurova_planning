from __future__ import print_function
import os, sys
import json
from copy import deepcopy

from PIL import Image
from collections import deque


import torch
import numpy as np
import math
import time
from sim2real_local_planner.model import *
from sim2real_local_planner.config import GlobalConfig
from torchvision import transforms as T

import itertools
import pathlib
import argparse
SAVE_PATH = os.environ.get('SAVE_PATH')

if not SAVE_PATH:
    SAVE_PATH = None
else:
    pathlib.Path(SAVE_PATH).mkdir(parents=True, exist_ok=True)

import zmq
from dataclasses import dataclass
#Dataclasses for imitate some carla objects

@dataclass
class Vector3D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
class Rotation:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
class BoundingBox:
    location: Vector3D
    extent: Vector3D
    rotation: Rotation

NEAR_COLLISION_DISTANCE = 1.8 # meters
NEAR_COLLISION_DISTANCE2 = 0.8 # Stop if obstacle is nearest than this distance from the sensor
MINIMUM_DISTANCE = 2.0
MAX_TIME_STOPPED = 5.0 #Stopped or local minimum


class Agent():
    def __init__(self, args):
        self.context = zmq.Context()
        self.socket_sub = self.context.socket(zmq.SUB)
        self.socket_sub.bind("tcp://*:5556")
        self.socket_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.bind("tcp://*:5555")
        self.publisher.setsockopt(zmq.LINGER, 500)
        self.status = 0
        self.last_steers=deque()
        self._im_transform = T.Compose([T.ToTensor(), T.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])])
        self._depth_transform = T.Compose([T.ToTensor(), T.Normalize(mean=[0.485], std=[0.229])])
        self.config = GlobalConfig()
        self.time_local_minimum=0
        self.previous_pose = [0,0]
        self.steer_local_minimum = 1.0 #Target steer for trying to exit from a local minimum. It alternates between minimum and maximum steering angle.
        self.safe_stop=False
        self.model = args.model

        # Load model file 
        state_dict = torch.load(args.model_path)
        #Remove the parameters cell and hidden
        try:
            del state_dict["hidden"], state_dict["cell"]
        except: 
            pass

        if self.model == "rgb": self.net = RGB_Network(self.config, False)
        elif self.model == "depth": self.net = Depth_Network(self.config, False)
        elif self.model == "rgb+depth": self.net = MTAP_Network(self.config, False)
        else: raise ValueError("Model not supported")
        
        self.net.to("cuda")
        self.net.load_state_dict(state_dict, strict = False)
        self.net.cuda()
        self.net.eval()

    def close_sockets(self):
        self.socket_sub.close()
        self.publisher.close()
        self.context.term()


    def tick(self):
        #For making the listener simpler, both images are received even if they are not used
        try:
            message = json.loads(self.socket_sub.recv())
            image=self.socket_sub.recv()
            depth=self.socket_sub.recv()
        except zmq.error.ZMQError as e:
            print(e)
            return None

        if message["end"]:
            # Instead of closing the program, reset the recurrent layer
            # exit(0)
            print("Initialize agent.")
            self.net.reset_RNN()
            return None

        speed=message["speed"]
        steer=message["steer"]
        target_point=message["target_point"]
        clock=message["clock"]

        result = {
                'speed': speed,
                'steer': steer,
                'target_point':target_point,
                'clock': clock
                }
        if self.model != "depth":
            img = np.frombuffer(image, np.uint8)
            img=np.reshape(img,(message["rgb_shape"]))
            img = img.swapaxes(0,2)
            result['rgb'] = img
        if self.model != "rgb":
            depth= np.frombuffer(depth,np.uint16)
            depth=np.reshape(depth,message["range_shape"])
            result['depth'] = depth

        return result

    @torch.inference_mode() # Faster version of torch_no_grad
    def run_step(self):  

        tick_data = self.tick()
        if tick_data is None: 
            self.publisher.send_string("End")
            return None
        # prepare image input
        image, depth = self.prepare_image(tick_data)

        # prepare state and goal input
        target_point = torch.tensor(tick_data['target_point']).to('cuda', dtype=torch.float32)
        speed = torch.tensor(tick_data['speed']).to('cuda',dtype=torch.float32).view(1)
        steer = torch.tensor(tick_data['steer']).to('cuda',dtype=torch.float32).view(1)
        steer *= -1
        target_point[1] *= -1
        state = torch.cat([speed, steer, target_point]).unsqueeze(0)
        

        # forward pass
        with torch.no_grad():
            if self.model == "rgb": pred = self.net.forward_inference(image, state, target_point.unsqueeze(0))
            elif self.model == "depth": pred = self.net.forward_inference(depth, state, target_point.unsqueeze(0))
            else: pred = self.net.forward_inference(image, depth, state, target_point.unsqueeze(0))
        pred['acker_speed_branches'],pred['acker_steer_branches'] = pred['acker_speed_branches'].data.cpu().numpy()[0][0],pred['acker_steer_branches'].data.cpu().numpy()[0][0]
        acker_speed_ctrl, acker_steer_ctrl = pred['acker_speed_branches']*self.config.max_speed, pred['acker_steer_branches']*self.config.max_steer
        pred_wp = pred['pred_wp'][0].data.cpu().numpy()
        #The Y axis in Carla/Unreal Engine has the inverse convention, transform for clearer estimation of angle and visualization
        pred_wp = np.array([[point[0], point[1]] for point in pred_wp])
        target_point = target_point.squeeze().data.cpu().numpy()
        # target_point[1] *= -1
        acker_speed_traj, acker_steer_traj = self.net.control_pid(pred_wp, target_point)
        acker_speed_traj = np.clip(acker_speed_traj, -self.config.max_speed, self.config.max_speed)
        acker_steer_traj = np.clip(acker_steer_traj, -self.config.max_steer, self.config.max_steer)

        if self.status == 0: self.alpha = 0.3
        else: self.alpha = 0.7
        self.alpha=1.0
        steer = self.alpha*acker_steer_ctrl + (1-self.alpha)*acker_steer_traj
        # speed = self.alpha*acker_speed_ctrl + (1-self.alpha)*acker_speed_traj
        speed = acker_speed_ctrl


        if len(self.last_steers) >= 20:
            self.last_steers.popleft()
        self.last_steers.append(abs(float(steer)))
        #chech whether ego is turning
        # num of steers larger than 0.1
        num = 0
        for s in self.last_steers:
            if s > 0.25*self.config.max_steer:
                num += 1
        self.status = 1 if num>10 else 0

        message={}
        message["speed"] = float(speed)
        steer*=-1
        message["steer"] = float(steer)
        
        d=math.sqrt((tick_data['target_point'][0]-self.previous_pose[0])**2+(tick_data['target_point'][1]-self.previous_pose[1])**2)
        #Try to avoid local minimum/command
        if d > MINIMUM_DISTANCE:
            self.time_local_minimum=tick_data['clock']
            self.previous_pose=tick_data['target_point']
        
        if tick_data['clock']-self.time_local_minimum>MAX_TIME_STOPPED:
            if abs(tick_data["steer"]-self.steer_local_minimum)<0.05:
                self.time_local_minimum=tick_data['clock']
                self.steer_local_minimum*=-1
            else:
                message["steer"]=self.config.max_steer*self.steer_local_minimum

        message["steer_ctrl"] = float(acker_steer_ctrl)
        message["speed_ctrl"] = float(acker_speed_ctrl)
        message["steer_traj"] = float(acker_steer_traj)
        message["speed_traj"] = float(acker_speed_traj)
        message["waypoints"] = pred_wp.tolist()

        self.publisher.send_string(json.dumps(message))

        return message


    def prepare_image(self, tick_data):
        image,depth=None, None
        
        if self.model != "depth":
            image=tick_data['rgb'].astype(np.uint8)
            width = image.shape[1]
            height = image.shape[0]
            crop_w, crop_h = self.config.input_rgb_resolution
            start_y = height // 2 - crop_h // 2
            start_x = width // 2 - crop_w // 2
            cropped_image = image[start_y:start_y + crop_h, start_x:start_x + crop_w]
            image=self._im_transform(cropped_image)
            image=image.to('cuda', dtype=torch.float32)
            image=image.unsqueeze(0)

        if self.model != "rgb":
            depth=tick_data['depth']
            height = depth.shape[1]
            width = depth.shape[0]
            crop_w, crop_h = self.config.input_range_resolution
            start_y = height // 2 - crop_h // 2
            start_x = width // 2 - crop_w // 2
            cropped_depth=depth[start_x:start_x + crop_w, start_y:start_y + crop_h]
            cropped_depth=self._depth_transform(cropped_depth.astype(np.float32))
            depth = cropped_depth.to('cuda', dtype=torch.float32)
            depth = torch.permute(depth,(0,2,1)).unsqueeze(0)
        return image, depth
    

    def destroy(self):
        del self.net
        self.close_sockets()

    
def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--model_path', type=str, required=True, help='Path to the model file')
    parser.add_argument('--model', type=str, default="rgb+depth", help='Model architecture to use (default: rgb+depth | rgb, depth, rgb+depth)')

    args = parser.parse_args()

    actor=Agent(args)
    print("Agent loaded")
    mean_time=0
    count=0
    try:
        while True:
            start=time.clock_gettime_ns(time.CLOCK_THREAD_CPUTIME_ID)
            actor.run_step()
            end=time.clock_gettime_ns(time.CLOCK_THREAD_CPUTIME_ID)
            mean_time+=(end-start)*10**-6
            count+=1
            # print("Required time (ms)", (end-start)*10**-6)
    finally:
        print("Required mean time (ms)", mean_time/count)
        actor.destroy()

if __name__ == '__main__':

    main()
