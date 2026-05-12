import argparse
import json
import os
from tqdm import tqdm
import cv2
import time

import numpy as np
import torch
import torch.distributed as dist
import torch.optim as optim
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from collections import OrderedDict
from torchvision import transforms as T

from MTAP_model import MTAP_Network
from config import GlobalConfig
from data import CARLA_Data


import pathlib
import datetime
from torch.distributed.elastic.multiprocessing.errors import record
import random
from torch.distributed.optim import ZeroRedundancyOptimizer
import torch.multiprocessing as mp

from diskcache import Cache
# Records error and tracebacks in case of failure
id=1
@record
def main():
    torch.cuda.empty_cache()

    parser = argparse.ArgumentParser()
    parser.add_argument('--model_file', type=str, help='model to load.')
    parser.add_argument('--output', type=str, default="results.csv", help='Output file name.')

    args = parser.parse_args()
    config = GlobalConfig()
    net = MTAP_Network(config)


    # Load model file
    # Load model file
    state_dict = torch.load(args.model_file)
    #Remove the parameters cell and hidden
    try:
        del state_dict["hidden"], state_dict["cell"]
    except: del state_dict["hidden_image"], state_dict["cell_image"], state_dict["hidden_pedestrian"], state_dict["cell_pedestrian"]	
    net.load_state_dict(state_dict, strict = False)
    print("Loaded model: ",args.model_file)
    net.to("cuda")
    net.eval()

    g_cuda = torch.Generator(device='cpu')
    g_cuda.manual_seed(torch.initial_seed())
    val_set = CARLA_Data(root=config.val_dir, config=config)

    dataloader_val   = dataloader_val = DataLoader(val_set, batch_size=1, shuffle=False, num_workers=1)
    # file = open(args.output, "w")
    file=None
    mean_time = 0
    count=0
    for data in tqdm(dataloader_val):
        mean_time+=load_data_and_save_waypoints(data, config, net, file)
        count+=1
    print("Mean time (ms)= ",(mean_time/count)*10**-6)
    # file.close()


def load_data_and_save_waypoints(data, config, net:MTAP_Network, file):
    global id
    net.reset_RNN()
    ellapsed_time = 0
    for i in range(len(data['image'][0])):
        start=time.clock_gettime_ns(time.CLOCK_THREAD_CPUTIME_ID)
        image = data['image'][:,i]
        depth = data['depth'][:,i].unsqueeze(1)
        image=torch.cat([image,depth], dim=1).to("cuda",dtype=torch.float32)
        speed = data['speed'][:,i].to("cuda",dtype=torch.float32).unsqueeze(-1)
        steer = data['steer'][:,i].to("cuda",dtype=torch.float32).unsqueeze(-1)

        target_point = data['target_point'][:,i].to("cuda",dtype=torch.float32)
        
        state = torch.cat([speed, steer, target_point], 1)

        gt_waypoints = data['waypoints'][:,i].data.cpu().numpy()[0]
        
        with torch.no_grad():
            pred = net.forward_inference(image, state, target_point)

        # pred_wp=pred["pred_wp"][0].data.cpu().numpy()
        # speed = pred['acker_speed_branches'][0].data.cpu().numpy()[0]
        # steer = pred['acker_steer_branches'][0].data.cpu().numpy()[0]
        # text = ""
        # gt_speed, gt_steer = data["action_acker_speed"][:,i].data.cpu().numpy()[0],data["action_acker_steer"][:,i].data.cpu().numpy()[0]
        # # for i in range(len(pred_wp)):
        # #     text+=str(pred_wp[i][0])+","+str(pred_wp[i][1])+","+str(gt_waypoints[i][0])+","+str(gt_waypoints[i][1])+","
        # # text=text+str(speed)+","+str(gt_speed)+","+str(steer)+","+str(gt_steer)+"\n"
        # # file.write(text)
        # pedestrian_att = pred["pedestrian_att"][0].data.cpu().numpy()[0]
        # pedestrian_att = (pedestrian_att*255)
        # pedestrian_att -= np.min(pedestrian_att)
        # pedestrian_att = pedestrian_att.astype(np.uint8)
        # pedestrian_att = cv2.resize(pedestrian_att,(2048,64),interpolation=cv2.INTER_NEAREST)
        # pedestrian_att = cv2.cvtColor(pedestrian_att,cv2.COLOR_GRAY2BGR)
        # att = pred["init_att"][0].data.cpu().numpy()[0]
        # att = (att*255)
        # att -= np.min(att)
        # att = att.astype(np.uint8)
        # att = cv2.resize(att,(2048,64),interpolation=cv2.INTER_NEAREST)
        # att = cv2.cvtColor(att,cv2.COLOR_GRAY2BGR)
        # image_val=data['image_val'][:,i].data.cpu().numpy()[0]
        # # image_val=image_val.swapaxes(0,1)
        # image_val = image_val.astype(np.uint8)
        # image_val=cv2.cvtColor(image_val,cv2.COLOR_RGB2BGR)
        # # image_val=np.vstack((pedestrian_att,image_val,att))
        # image_val=np.vstack((image_val,att))
        # cv2.imwrite("results/attention/"+str(id)+".png",image_val)
        # id+=1
        end=time.clock_gettime_ns(time.CLOCK_THREAD_CPUTIME_ID)
        ellapsed_time+=end-start
    print("Time (ms)= ",(ellapsed_time/len(data['image'][0]))*10**-6)
    return ellapsed_time/len(data['image'][0])



# We need to seed the workers individually otherwise random processes in the dataloader return the same values across workers!
def seed_worker(worker_id):
    # Torch initial seed is properly set across the different workers, we need to pass it to numpy and random.
    worker_seed = (torch.initial_seed()) % 2**32
    np.random.seed(worker_seed)
    random.seed(worker_seed)


if __name__ == "__main__":
    # The default method fork can run into deadlocks.
    # To use the dataloader with multiple workers forkserver or spawn should be used.
    mp.set_start_method('fork')
    print("Start method of multiprocessing:", mp.get_start_method())
    main()