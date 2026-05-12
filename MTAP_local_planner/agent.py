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
from MTAP_model import *
from config import GlobalConfig
from torchvision import transforms as T

import itertools
import pathlib
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
    def __init__(self, model_file):
        self.context = zmq.Context()
        self.socket_sub = self.context.socket(zmq.SUB)
        self.socket_sub.bind("tcp://*:5556")
        self.socket_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.bind("tcp://*:5555")
        self.publisher.setsockopt(zmq.LINGER, 500)
        self.config_path = model_file
        self.status = 0
        self.last_steers=deque()
        self._im_transform = T.Compose([T.ToTensor(), T.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])])
        self._depth_transform = T.Compose([T.ToTensor(), T.Normalize(mean=[0.485], std=[0.229])])
        self.config = GlobalConfig()
        self.time_local_minimum=0
        self.previous_pose = [0,0]
        self.steer_local_minimum = 1.0 #Target steer for trying to exit from a local minimum. It alternates between minimum and maximum steering angle.
        self.safe_stop=False

        # Load model file 
        state_dict = torch.load(model_file)
        #Remove the parameters cell and hidden
        try:
            del state_dict["hidden"], state_dict["cell"]
        except: 
            pass
        if "seg_decoder.deconv1.0.weight" in state_dict: self.net = MTAP_Network(self.config)
        else: self.net = MTAP_Network_without_Semantic(self.config)
        self.net.to("cuda")
        self.net.load_state_dict(state_dict, strict = False)
        self.net.cuda()
        self.net.eval()

    def close_sockets(self):
        self.socket_sub.close()
        self.publisher.close()
        self.context.term()


    def tick(self):
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
        img = np.frombuffer(image)
        img=np.reshape(img,message["image_shape"])
        depth= np.frombuffer(depth,np.float32)
        #The depth image has one channel
        depth=np.reshape(depth,[message["image_shape"][0],message["image_shape"][1],1])

        speed=message["speed"]
        steer=message["steer"]
        target_point=message["target_point"]
        clock=message["clock"]

        result = {
                'rgb': img,
                'depth': depth,
                'speed': speed,
                'steer': steer,
                'target_point':target_point,
                'clock': clock
                }

        return result

    @torch.inference_mode() # Faster version of torch_no_grad
    def run_step(self):  

        tick_data = self.tick()
        if tick_data is None: 
            self.publisher.send_string("End")
            return None
        # prepare image input
        image = self.prepare_image(tick_data)

        # prepare state and goal input
        target_point = torch.tensor(tick_data['target_point']).to('cuda', dtype=torch.float32)
        speed = torch.tensor(tick_data['speed']).to('cuda',dtype=torch.float32).view(1)
        steer = torch.tensor(tick_data['steer']).to('cuda',dtype=torch.float32).view(1)
        state = torch.cat([speed, steer, target_point]).unsqueeze(0)

        # forward pass
        with torch.no_grad():
            pred = self.net.forward_inference(image, state, target_point.unsqueeze(0))

        pred['acker_speed_branches'],pred['acker_steer_branches'] = pred['acker_speed_branches'].data.cpu().numpy()[0][0],pred['acker_steer_branches'].data.cpu().numpy()[0][0]
        acker_speed_ctrl, acker_steer_ctrl = pred['acker_speed_branches']*self.config.max_speed, pred['acker_steer_branches']*self.config.max_steer
        pred_wp = pred['pred_wp'][0].data.cpu().numpy()
        #The Y axis in Carla/Unreal Engine has the inverse convention, transform for clearer estimation of angle and visualization
        pred_wp = np.array([[point[0], -point[1]] for point in pred_wp])
        target_point = target_point.squeeze().data.cpu().numpy()
        target_point[1] *= -1
        acker_speed_traj, acker_steer_traj = self.net.control_pid(pred_wp, target_point)

        if self.status == 0: self.alpha = 0.3
        else: self.alpha = 0.7
        self.alpha=1.0
        steer = self.alpha*acker_steer_ctrl + (1-self.alpha)*acker_steer_traj
        speed = self.alpha*acker_speed_ctrl + (1-self.alpha)*acker_speed_traj


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
        try:
            semantic = pred["seg_img"].data.cpu().numpy()
            converter = np.array(self.config.classes_list)
            indices = np.argmax(semantic[0], axis=0)
            semantic = converter[indices].astype('uint8').tobytes()
        except:
            semantic=np.zeros([128,2048,3]).astype('uint8').tobytes()

        self.publisher.send_string(json.dumps(message),zmq.SNDMORE)
        self.publisher.send(semantic)
        return message


    def prepare_image(self, tick_data):
        image=tick_data['rgb'].astype(np.uint8)
        height = image.shape[1]
        width = image.shape[0]
        crop_w, crop_h = self.config.input_resolution
        start_y = height // 2 - crop_h // 2
        start_x = width // 2 - crop_w // 2

        cropped_image = image[start_x:start_x + crop_w, start_y:start_y + crop_h]
        image=self._im_transform(cropped_image)
        depth=tick_data['depth']
        cropped_depth=depth[start_x:start_x + crop_w, start_y:start_y + crop_h]
        # cropped_depth=self._depth_transform(cropped_depth)
        cropped_depth=torch.tensor(cropped_depth).permute((2,0,1))
        image = torch.cat([image,cropped_depth])
        image=image.to('cuda', dtype=torch.float32)
        image=torch.permute(image,(0,2,1))
        return image.unsqueeze(0)

    def scale_crop(self, image, scale=1, start_x=0, crop_x=None, start_y=0, crop_y=None):
        (width, height) = (image.width // scale, image.height // scale)
        if scale != 1:
            image = image.resize((width, height))
        if crop_x is None:
            crop_x = width
        if crop_y is None:
            crop_y = height
            
        image = np.asarray(image)
        cropped_image = image[start_y:start_y+crop_y, start_x:start_x+crop_x]
        return cropped_image

    def shift_x_scale_crop(self, image, scale, crop, crop_shift=0):
        crop_h, crop_w = crop
        (width, height) = (int(image.width // scale), int(image.height // scale))
        im_resized = image.resize((width, height))
        image = np.array(im_resized)
        start_y = height//2 - crop_h//2
        start_x = width//2 - crop_w//2
        
        # only shift in x direction
        start_x += int(crop_shift // scale)
        cropped_image = image[start_y:start_y+crop_h, start_x:start_x+crop_w]
        cropped_image = np.transpose(cropped_image, (2,0,1))
        return cropped_image
    
    # def check_near_collision(self,depth, reverse=False):
    #     min_distance=1
    #     if reverse:
    #         for col in itertools.chain(range(0,250), range(1798,2048)):
    #             for row in range(64,128):
    #                 if depth[col,row]<min_distance:
    #                     min_distance=depth[col,row]
    #     else:
    #         for col in range(750,1250):
    #             for row in range(64,128):
    #                 if depth[col,row]<min_distance:
    #                     min_distance=depth[col,row]
    #     return float(min_distance)
    

    def destroy(self):
        del self.net
        self.close_sockets()

    
def main():
    if(len(sys.argv)>1):
        model_file=sys.argv[1]
        if(not os.path.isfile(model_file)):
            print("ERROR: incorrect filename. Usage: python agent.py path_to_model_file")
            exit()
    else:
        print("ERROR: Provide the model file. Usage: python agent.py path_to_model_file")
        exit()
    actor=Agent(model_file)
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
