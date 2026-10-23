import argparse
import json
import os
from tqdm import tqdm
import cv2

import numpy as np
import torch
import torch.distributed as dist
import torch.optim as optim
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from collections import OrderedDict
from torchvision import transforms as T
import torch.nn.functional as F

from sim2real_local_planner.model import Safety_MTAP_Network
from sim2real_local_planner.config import GlobalConfig
from sim2real_local_planner.data import CARLA_Data


import pathlib
import datetime
from torch.distributed.elastic.multiprocessing.errors import record
import random
from torch.distributed.optim import ZeroRedundancyOptimizer
import torch.multiprocessing as mp
from sim2real_local_planner.loss import focal_R_loss

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
    net = Safety_MTAP_Network(config)


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
    val_set = CARLA_Data(root=config.val_dir, config=config, model="safety")

    dataloader_val   = dataloader_val = DataLoader(val_set, batch_size=4, shuffle=False, num_workers=1)
    # file = open(args.output, "w")
    file=None
    for data in tqdm(dataloader_val):
        load_data_and_save_waypoints(data, config, net, file)
        break
    # file.close()


def load_data_and_save_waypoints(data, config, net:Safety_MTAP_Network, file):
    global id
    net.reset_RNN()
    image = data['image'].to("cuda",dtype=torch.float32)
    depth = data['depth'].to("cuda",dtype=torch.float32)
    speed = data['speed'].to("cuda",dtype=torch.float32).unsqueeze(-1)
    steer = data['steer'].to("cuda",dtype=torch.float32).unsqueeze(-1)
    target_point = data['target_point'].to("cuda",dtype=torch.float32)
    
    state = torch.cat([speed, steer, target_point], 2).to("cuda")

    
    with torch.no_grad():
        pred = net(image, depth, state, target_point)

    safety_matrix = pred["safety_matrix"].data.cpu().numpy()
    gt_safety_matrix = data["safety_matrix"].data.cpu().numpy()
    for j in range(len(safety_matrix)):
        print("Safety matrix: ",j, " ", safety_matrix[j][0], " GT: ", gt_safety_matrix[j][0],"\n")
        print(F.l1_loss(torch.tensor(safety_matrix[j][0]), torch.tensor(gt_safety_matrix[j][0])))
        print(F.l1_loss(torch.tensor(safety_matrix[j]), torch.tensor(gt_safety_matrix[j])))
        




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