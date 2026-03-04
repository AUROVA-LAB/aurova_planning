import argparse
import os

import torch
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.data import DataLoader
import random
import numpy as np

from torch.utils.tensorboard import SummaryWriter

from model import *
from loss import *
from data import CARLA_Data
from config import GlobalConfig
from tqdm import tqdm
from collections import OrderedDict
from typing import Union
import time

class Engine(object):
	"""
    Engine that runs training.
    """

	def __init__(self, model: MTAP_Network, dataloader_train, 
			  dataloader_val, args, config, version, device="cuda", cur_epoch=1):
		self.cur_epoch = cur_epoch
		self.save_top_k = args.save_k
		self.top_k = [None for i in range(self.save_top_k)]
		self.last_name=None
		self.bestval = [1e10 for i in range(self.save_top_k)]
		self.model = model
		model.to(device)
		self.optimizer = optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-7)
		# self.lr_scheduler = optim.lr_scheduler.StepLR(self.optimizer, 5, 0.5)
		self.dataloader_train = dataloader_train
		self.dataloader_val   = dataloader_val
		self.args = args
		self.config = config
		self.logger = SummaryWriter(version)
		self.device = device
		if type(self.model) == Safety_MTAP_Network:
			self.actions_matrix = torch.zeros((self.config.num_actions_steer, self.config.num_actions_speed,2), dtype=torch.float32).to(self.device)
			for i in range(self.config.num_actions_speed):
				for j in range(self.config.num_actions_steer):
					self.actions_matrix[j,i,0] = 2*i/(self.config.num_actions_speed-1)-1
					self.actions_matrix[j,i,1] = 2*j/(self.config.num_actions_steer-1)-1
					# Flatten actions_matrix to (num_actions, 2)
			self.actions_matrix = self.actions_matrix.view(-1, 2)  # shape: (n*m, 2)
	def fit(self):
		for epoch in range(self.cur_epoch, args.epochs+1):
			self.cur_epoch=epoch
			loss_function_ctrl, loss_fuction_wp = [focal_R_loss, focal_R_loss] 
			# loss_function_ctrl, loss_fuction_wp = [F.l1_loss,F.l1_loss]
			# Training loop
			
			self.model.train()
			metrics_epoch=None
			for data in tqdm(self.dataloader_train, desc="Training epoch {}".format(epoch), leave=False):
				self.optimizer.zero_grad(set_to_none=True,)
				
				loss,metrics=self.training_step(data, loss_function_ctrl, loss_fuction_wp)

				loss.backward()
				self.optimizer.step()

				if metrics_epoch is None: metrics_epoch=metrics
				else:
					for key,value in metrics.items():
						metrics_epoch[key] += float(value) 

			# self.lr_scheduler.step()
			print("Training epoch {} loss={:.5f}".format(epoch,metrics_epoch["train_loss"]/len(self.dataloader_train)))
			for key,value in metrics_epoch.items():
				self.logger.add_scalar(key,value/len(self.dataloader_train), self.cur_epoch)
			


			########Validation loop#########			
			metrics_val_epoch=None
			self.model.eval()
			for data in tqdm(self.dataloader_val, desc="Validation epoch {}".format(epoch),leave=False):
				
				metrics = self.validation_step(data, loss_function_ctrl, loss_fuction_wp)

				if metrics_val_epoch is None: metrics_val_epoch=metrics
				else:
					for key,value in metrics.items():
						metrics_val_epoch[key] += float(value)


			print("Validation epoch {} loss={:.5f}".format(epoch,metrics_val_epoch["val_loss"]/len(self.dataloader_val)))
			for key,value in metrics_val_epoch.items():
				self.logger.add_scalar(key,value/len(self.dataloader_val), self.cur_epoch)

			#Save last model and the best k models
			metrics_val_epoch["val_loss"] /= len(self.dataloader_val)
			if self.last_name is not None:
				os.remove(self.logger.log_dir+self.last_name)
			self.last_name="/last-val_loss{:.5f}.pth".format(metrics_val_epoch["val_loss"])
			torch.save(self.model.state_dict(),self.logger.log_dir+self.last_name)
			for i in range(self.save_top_k):
				if metrics_val_epoch["val_loss"]<self.bestval[i]:
					self.bestval.insert(i,metrics_val_epoch["val_loss"])
					self.bestval=self.bestval[:self.save_top_k]
					name = "/epoch{}-val_loss{:.5f}.pth".format(epoch,metrics_val_epoch["val_loss"])
					self.top_k.insert(i, name)
					if self.top_k[-1] is not None:
						os.remove(self.logger.log_dir+self.top_k[-1])
					torch.save(self.model.state_dict(),self.logger.log_dir+name)
					self.top_k=self.top_k[:self.save_top_k]
					break

	def training_step(self, batch, loss_function_ctrl, loss_fuction_wp):
		if type(self.model) != Depth_Network: image = batch['image'].to(self.device,dtype=torch.float32)
		if type(self.model) != RGB_Network: depth = batch['depth'].to(self.device,dtype=torch.float32)
		speed = batch['speed'].to(self.device,dtype=torch.float32).unsqueeze(-1)
		steer = batch['steer'].to(self.device,dtype=torch.float32).unsqueeze(-1)
		target_point = batch['target_point'].to(self.device,dtype=torch.float32)
		batch['action_acker_speed']=batch['action_acker_speed'].to(self.device)
		batch['action_acker_steer']=batch['action_acker_steer'].to(self.device)
		batch['future_action_acker_speed']=batch['future_action_acker_speed'].to(self.device)
		batch['future_action_acker_steer']=batch['future_action_acker_steer'].to(self.device)

		state = torch.cat([speed, steer, target_point], dim=2).to(self.device)


		gt_waypoints = batch['waypoints'].to(self.device)
		
		if type(self.model) == RGB_Network: pred = self.model(image, state, target_point)
		elif type(self.model) == Depth_Network: pred = self.model(depth, state, target_point)
		else: pred = self.model(image, depth, state, target_point)
		
		action_loss = loss_function_ctrl(pred['acker_speed_branches'],batch['action_acker_speed'].unsqueeze(-1)) * 0.5 + \
			loss_function_ctrl(pred['acker_steer_branches'],batch['action_acker_steer'].unsqueeze(-1)) *0.5
		

		future_action_loss =  loss_function_ctrl(pred['future_acker_speed'], batch['future_action_acker_speed'].unsqueeze(-1)) * 0.5 + \
							loss_function_ctrl(pred['future_acker_steer'], batch['future_action_acker_steer'].unsqueeze(-1)) * 0.5
		wp_loss = loss_fuction_wp(pred['pred_wp'], gt_waypoints)

		loss = action_loss  + wp_loss + future_action_loss

		metrics = {'train_action_loss':action_loss.item(),
			 		'train_wp_loss_loss': wp_loss.item(),
					'train_future_action_loss': future_action_loss.item(),
					'train_loss': loss.item()}
		
		if type(self.model) == Safety_MTAP_Network:
			batch['safety_matrix']=batch['safety_matrix'].to(self.device)
			safety_matrix_loss = focal_R_loss(pred['safety_matrix'], batch['safety_matrix'])
			# Compute the distance between the predicted action and each action in the actions_matrix
			# Stack predicted speed and steer into shape (batch, 2)
			pred_actions = torch.cat([pred['acker_speed_branches'], pred['acker_steer_branches']], dim=-1)  # shape: (batch,seq, 2)
			# Compute distance from each predicted action to all actions in the matrix
			# pred_actions: (batch, 2), self.actions_matrix: (n*m, 2)
			# Where n is the number of speed actions and m the steer actions.
			# Expand pred_actions for broadcasting
			pred_actions_expanded = pred_actions.unsqueeze(2)  # (batch,seq, 1, 2)
			actions_matrix_expanded = self.actions_matrix.unsqueeze(0)  # (1, n*m, 2)
			action_distance = torch.linalg.norm(pred_actions_expanded - actions_matrix_expanded, dim=-1)  # (batch,seq, n*m)
			safety_loss = torch.sum(softmin_temperature(action_distance, 0.05)*batch["safety_matrix"],dim=-1).mean()
			loss += safety_loss + safety_matrix_loss
			metrics["train_safety_matrix_loss"]=safety_matrix_loss.item()
			metrics["train_safety_loss"]=safety_loss.item()
			metrics["train_loss"]=loss.item()

		return loss, metrics
	
	def validation_step(self, batch, loss_function_ctrl, loss_fuction_wp):
		
		if type(self.model) != Depth_Network: image = batch['image'].to(self.device,dtype=torch.float32)
		if type(self.model) != RGB_Network: depth = batch['depth'].to(self.device,dtype=torch.float32)
		speed = batch['speed'].to(self.device,dtype=torch.float32).unsqueeze(-1)
		steer = batch['steer'].to(self.device,dtype=torch.float32).unsqueeze(-1)
		target_point = batch['target_point'].to(self.device,dtype=torch.float32)
		batch['action_acker_speed']=batch['action_acker_speed'].to(self.device)
		batch['action_acker_steer']=batch['action_acker_steer'].to(self.device)
		batch['future_action_acker_speed']=batch['future_action_acker_speed'].to(self.device)
		batch['future_action_acker_steer']=batch['future_action_acker_steer'].to(self.device)
		
		state = torch.cat([speed, steer, target_point], 2).to(self.device)

		gt_waypoints = batch['waypoints'].to(self.device)
		
		if type(self.model) == RGB_Network: pred = self.model(image, state, target_point)
		elif type(self.model) == Depth_Network: pred = self.model(depth, state, target_point)
		else: pred = self.model(image, depth, state, target_point)
		#Gradients aren't used.
		for key in pred.keys():
			pred[key] = pred[key].detach()
		action_loss = loss_function_ctrl(pred['acker_speed_branches'],batch['action_acker_speed'].unsqueeze(-1)) * 0.5 + \
			loss_function_ctrl(pred['acker_steer_branches'],batch['action_acker_steer'].unsqueeze(-1)) *0.5
		

		future_action_loss =  loss_function_ctrl(pred['future_acker_speed'], batch['future_action_acker_speed'].unsqueeze(-1)) * 0.5 + \
							loss_function_ctrl(pred['future_acker_steer'], batch['future_action_acker_steer'].unsqueeze(-1)) * 0.5
		wp_loss = loss_fuction_wp(pred['pred_wp'], gt_waypoints)

		loss = action_loss  + wp_loss + future_action_loss

		metrics = {'val_action_loss':action_loss.item(),
			 		'val_wp_loss_loss': wp_loss.item(),
					'val_future_action_loss': future_action_loss.item(),
					'val_loss': loss.item()}
		
		if type(self.model) == Safety_MTAP_Network:
			batch['safety_matrix']=batch['safety_matrix'].to(self.device)
			safety_matrix_loss = focal_R_loss(pred['safety_matrix'], batch['safety_matrix'])
			# Compute the distance between the predicted action and each action in the actions_matrix
			# Stack predicted speed and steer into shape (batch, 2)
			pred_actions = torch.cat([pred['acker_speed_branches'], pred['acker_steer_branches']], dim=-1)  # shape: (batch, 2)
			# Compute distance from each predicted action to all actions in the matrix
			# pred_actions: (batch, 2), self.actions_matrix: (n*m, 2)
			# Where n is the number of speed actions and m the steer actions.
			# Expand pred_actions for broadcasting
			pred_actions_expanded = pred_actions.unsqueeze(2)  # (batch,seq, 1, 2)
			actions_matrix_expanded = self.actions_matrix.unsqueeze(0)  # (1, n*m, 2)
			action_distance = torch.linalg.norm(pred_actions_expanded - actions_matrix_expanded, dim=-1)  # (batch,seq, n*m)
			safety_loss = torch.sum(softmin_temperature(action_distance, 0.05)*batch["safety_matrix"],dim=-1).mean()
			loss += safety_loss + safety_matrix_loss
			metrics["val_safety_matrix_loss"]=safety_matrix_loss.item()
			metrics["val_safety_loss"]=safety_loss.item()
			metrics["val_loss"]=loss.item()

		return metrics


if __name__ == "__main__":
	parser = argparse.ArgumentParser()

	parser.add_argument('--id', type=str, default='network2', help='Unique experiment identifier.')
	parser.add_argument('--epochs', type=int, default=60, help='Number of train epochs.')
	parser.add_argument('--start_epoch', type=int, default=1, help='Number of the starting epoch (for continuing previous training)')
	parser.add_argument('--weights', type=str, default=None, help='Load weights (for continuing previous training)')
	parser.add_argument('--lr', type=float, default=0.0001, help='Learning rate.')
	parser.add_argument('--batch_size', type=int, default=32, help='Batch size')
	parser.add_argument('--logdir', type=str, default='log', help='Directory to log data to.')
	parser.add_argument('--save_k', type=int, default=1, help='Number of top models to save')
	parser.add_argument('--num_workers', type=int, default=1, help='Number of workers to load the data')
	parser.add_argument('--model', type=str, default="rgb+depth", help='Model architecture to use (default: rgb+depth | rgb, depth, rgb+depth, safety)')

	

	args = parser.parse_args()
	args.logdir = os.path.join(args.logdir, args.id)

	# Config
	config = GlobalConfig()


	if args.model == "rgb+depth":
		model=MTAP_Network(config)
	elif args.model == "rgb":
		model=RGB_Network(config)
	elif args.model == "depth":
		model=Depth_Network(config)
	elif args.model == "safety":
		model=Safety_MTAP_Network(config)
	else:
		print("Error model not found: ",args.model)
		print("Valid models: rgb+depth, rgb, depth")
		exit()

	# Data
	train_set = CARLA_Data(root=config.train_dir, config=config, model=args.model)
	val_set = CARLA_Data(root=config.val_dir, config=config, model=args.model)

	dataloader_train = DataLoader(train_set, batch_size=args.batch_size, shuffle=True, num_workers=args.num_workers,persistent_workers=True)
	dataloader_val = DataLoader(val_set, batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers,persistent_workers=True)

	if not os.path.exists(args.logdir): os.makedirs(args.logdir, exist_ok=True)
	i=0
	while True:
		if not os.path.exists(args.logdir+"/version_"+str(i)):
			version=args.logdir+"/version_"+str(i)
			break
		i+=1

	if (args.weights is not None):
		# Load model file
		state_dict = torch.load(args.weights)
		#Remove the parameters cell and hidden
		try:
			del state_dict["hidden"], state_dict["cell"]	
		except:
			pass
		model.load_state_dict(state_dict, strict = False)
		print("Loaded model: ",args.weights)
	trainer = Engine(model,dataloader_train,dataloader_val,args,config,version,"cuda", args.start_epoch)

	print("--------------training-------------")
	try:
		trainer.fit()
	finally:
		del(trainer)



		




