import os

import torch
import torch.optim as optim
import torch.nn.functional as F
import random
import numpy as np

from torch.utils.tensorboard import SummaryWriter
#from pytorch_lightning.plugins import DDPPlugin

from copycat_model import CopycatNetwork
from data import Action_Data
from config import GlobalConfig
from tqdm import tqdm
from collections import OrderedDict
import time

class Engine(object):
	"""
    Engine that runs training.
    """

	def __init__(self, model, dataset_train, dataset_val, config, device="cuda"):
		self.model = model
		model.to(device)
		self.optimizer = optim.Adam(model.parameters(), lr=0.0001, weight_decay=1e-7)
		self.dataset_train = dataset_train
		self.dataset_val   = dataset_val
		self.config = config
		self.device = device
	
	def fit(self):
		for epoch in range(20):
			# Training loop
			# Create random order of the sequences for this loop
			
			self.model.train()

			for route in tqdm(self.dataset_train, desc="Training epoch {}".format(epoch), leave=False):
				self.model.reset_RNN()
				for data in tqdm(route, leave=False):
					self.optimizer.zero_grad(set_to_none=True,)
					loss=self.step(data)

					loss.backward()
					self.optimizer.step()

	def save_weigths(self):
		self.model.eval()

		for route in tqdm(self.dataset_train, desc="Saiving weigths (train dataset)", leave=False):
			self.model.reset_RNN()
			losses=[]
			for data in tqdm(route, leave=False):
				losses.append(self.step(data))
			
			try:
				os.remove(route.output_dir)
			except OSError:
				pass

			np.save(route.output_dir, np.array(losses))

		for route in tqdm(self.dataset_val, desc="Saiving weights (validation dataset)", leave=False):
			self.model.reset_RNN()
			losses=[]
			for data in tqdm(route, leave=False):
				losses.append(self.step(data))
			
			try:
				os.remove(route.output_dir)
			except OSError:
				pass

			np.save(route.output_dir, np.array(losses))





	def step(self, data):
		input=dict()
		input=torch.tensor([data['speed'],data['steer']]).to(self.device)
		data['action_acker_speed']=data['action_acker_speed'].to(self.device)
		data['action_acker_steer']=data['action_acker_steer'].to(self.device)
		data['future_action_acker_speed']=data['future_action_acker_speed'].to(self.device)
		data['future_action_acker_steer']=data['future_action_acker_steer'].to(self.device)
		data['waypoints'] = data['waypoints'].to(self.device)
		
		pred = self.model(input)

		#THE PAPER USES MSE!!!
		action_loss = F.l1_loss(pred['acker_speed_branches'],data['action_acker_speed']) * 0.5 + \
			F.l1_loss(pred['acker_speed_branches'],data['action_acker_steer']) *0.5

		future_action_loss =  F.l1_loss(pred['future_acker_speed'], data['future_action_acker_speed']) * 0.5 + \
							F.l1_loss(pred['future_acker_steer'], data['future_action_acker_steer']) * 0.5
		wp_loss = F.l1_loss(pred['pred_wp'], data['waypoints'].view(-1))

		loss = action_loss + wp_loss + future_action_loss 

		return loss

if __name__ == "__main__":
	# Config
	config = GlobalConfig()

	# Data
	train_set = Action_Data(root=config.train_dir, config=config)
	val_set = Action_Data(root=config.val_dir, config=config)

	model = CopycatNetwork(config.pred_len)
	model.float()
	
	trainer = Engine(model,train_set,val_set,config,"cuda")

	
	try:
		trainer.fit()
		trainer.save_weigths()
	finally:
		del(trainer)



		




