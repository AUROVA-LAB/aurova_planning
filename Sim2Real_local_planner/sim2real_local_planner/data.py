import os
from pathlib import Path
from PIL import Image
import cv2
import numpy as np
import torch 
from torch.utils.data import Dataset
from torchvision import transforms as T
from scipy.ndimage import interpolation
from tqdm import tqdm
import sys
import csv

class CARLA_Data(Dataset):

	def __init__(self, root, config, model):
		self.img_resolution = config.input_rgb_resolution
		self.range_resolution = config.input_range_resolution
		self.scale = config.scale
		self.augment_reflect = config.augment_reflect
		self.prob_color_jitter = config.prob_color_jitter 
		self.generator=np.random.default_rng()
		self.prob_localization_disturb = config.prob_localization_disturb
		self.mean_error_localization = config.mean_error_localization
		self.std_error_localization = config.std_error_localization
		self.prob_state_disturb = config.prob_state_disturb
		self.mean_error_state = config.mean_error_state
		self.std_error_state = config.std_error_state
		self.max_speed=config.max_speed
		self.max_steer=config.max_steer
		self.pred_length = config.pred_len
		self.sequence_size=config.sequence_size
		self.model=model
		self.num_actions_speed = config.num_actions_speed
		self.num_actions_steer = config.num_actions_steer
		
		self.color_jitter = T.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.3, hue=0.2)
		self._im_transform = T.Compose([T.ToTensor(), T.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])])
		self._depth_transform = T.Compose([T.ToTensor(), T.Normalize(mean=[0.485], std=[0.229])])

		self.images = []
		self.depth_img = []
		self.measurements = []
		self.commands = []
		self.augment_indices = []
		self.safety_matrices = []

		self.routes=[]

		data = os.listdir(root)
		for route in data:
			if any(scene in route for scene in config.selected_scenes):
				self.routes.append(os.path.join(root, route))

		count=0
		zero_command, self.zero_image = None, None
		for route in tqdm(self.routes, file=sys.stdout):
			
			route_dir = Path(route)

			route_len = len(os.listdir(route_dir / config.rgb_path))
			route_measurements = []
			route_commands = []
			route_safety_matrices = []

			measurement= route_dir / "measurements.csv"
			with open(measurement) as csvfile:
				file = csv.DictReader(csvfile)
				for row in file:
					route_measurements.append(row)

			command = route_dir / "commands.csv"
			with open(command) as csvfile:
				file = csv.DictReader(csvfile)
				for row in file:
					route_commands.append(row)

			safety_matrices = route_dir / "safety_matrices.txt"
			with open(safety_matrices) as csvfile:
				file = csv.reader(csvfile)
				for row in file:
					route_safety_matrices.append(row)

			# Loads by sequence
			seq_id=0
			seq_images, seq_depth_img = [],[]
			for idx in range(1,route_len+1):
				count+=1
				seq_images.append(route_dir / config.rgb_path / ("%d.png" % (idx)))
				seq_depth_img.append(route_dir / "lidar" / ("%d.png" % (idx)))
				seq_id+=1
				if seq_id==self.sequence_size:
					if idx+self.pred_length<=len(route_measurements):
						self.measurements.append(route_measurements[idx-seq_id:idx+self.pred_length])
						self.commands.append(route_commands[idx-seq_id:idx+self.pred_length])
						self.safety_matrices.append(route_safety_matrices[idx-seq_id:idx+self.pred_length])
					else:
						self.measurements.append(route_measurements[idx-seq_id:])
						self.commands.append(route_commands[idx-seq_id:])
						self.safety_matrices.append(route_safety_matrices[idx-seq_id:])
					self.images.append(seq_images)
					self.depth_img.append(seq_depth_img)
					self.augment_indices.append(False)
					if self.augment_reflect:
						self.measurements.append(self.measurements[-1])
						self.commands.append(self.commands[-1])
						self.safety_matrices.append(self.safety_matrices[-1])
						self.images.append(seq_images)
						self.depth_img.append(seq_depth_img)
						self.augment_indices.append(True)
					seq_id=0
					seq_images, seq_depth_img, seq_seg_img = [],[], []
			if seq_id>0:
				if zero_command is None:
					zero_command = {}
					for key in route_commands[0]:
						zero_command[key]=0.0
				last_measurement=route_measurements[-1]
				last_safety_matrix=route_safety_matrices[-1]
				while seq_id<self.sequence_size:
					route_measurements.append(last_measurement)
					route_commands.append(zero_command)
					route_safety_matrices.append(last_safety_matrix)
					seq_images.append("None")
					seq_depth_img.append("None")
					seq_seg_img.append("None")
					seq_id += 1
				self.measurements.append(route_measurements[-self.sequence_size:])
				self.commands.append(route_commands[-self.sequence_size:])
				self.safety_matrices.append(route_safety_matrices[-self.sequence_size:])
				self.images.append(seq_images)
				self.depth_img.append(seq_depth_img)
				self.augment_indices.append(False)
				#For augmentation, we consider only fliping the data, obtaining the double of sequences. 
				#Indicate to perform the augmentation while loading the sequence.
				if self.augment_reflect:
						self.measurements.append(self.measurements[-1])
						self.commands.append(self.commands[-1])
						self.safety_matrices.append(self.safety_matrices[-1])
						self.images.append(seq_images)
						self.depth_img.append(seq_depth_img)
						self.augment_indices.append(True)

			
			assert(len(self.measurements)==len(self.commands) and len(self.measurements)==len(self.images))
		
		self.num_sequences=len(self.images)
		if self.augment_reflect: count *= 2
		self.images       = np.array(self.images      ).astype(np.str)
		self.depth_img       = np.array(self.depth_img      ).astype(np.str)
		print("Loading %d frames from %d folders"%(count, len(self.routes)))
		print("Adding %d empty frames to compleate the sequences"%(self.num_sequences*config.sequence_size-count))
		print("Number of sequences of size %d: %d"%(config.sequence_size,self.num_sequences))


	def __len__(self):
		"""Returns the length of the dataset. """
		return self.num_sequences

	def __getitem__(self, index):
		"""Returns the item at index idx. """
		data = dict()
		augment_reflect=self.augment_indices[index]
		keys=['image','depth','waypoints','action_acker_speed','action_acker_steer',
			  'future_action_acker_speed','future_action_acker_steer','target_point','speed','steer', 'safety_matrix']
		if self.model=="rgb":keys.remove("depth")
		elif self.model=="depth":
			keys.remove("image")
		for key in keys:
			data[key] = []
		for i in range(self.sequence_size):

			#The None value is converted to string when converting to a np.array
			if self.images[index][i]=="None":
				if self.model!="depth": 
					data['image'].append(self.zero_image)
				if self.model!="rgb": data['depth'].append(self.zero_depth)
			else:
				if self.model!="depth":
					img = Image.open(self.images[index][i])
					if augment_reflect: img = img.transpose(Image.FLIP_LEFT_RIGHT)
					img = scale_and_crop_image(img, scale=self.scale, crop_w=self.img_resolution[0], crop_h=self.img_resolution[1])
					if np.random.rand()<self.prob_color_jitter:
						img = self.color_jitter(img)
					img=self._im_transform(img).type(torch.float32)
					data['image'].append(img)
					

				if self.model!="rgb":
					depth = cv2.imread(self.depth_img[index][i],cv2.IMREAD_UNCHANGED)
					depth = depth.swapaxes(0,1)
					if augment_reflect: depth = np.flip(depth, 0)
					depth = scale_and_crop_numpy(depth, scale=1, crop_w=self.range_resolution[0], crop_h=self.range_resolution[1]).swapaxes(0,1)
					depth=self._depth_transform(depth.astype(np.float32)).type(torch.float32)
					data['depth'].append(depth)

				if self.zero_image is None:
					if self.model!="depth":
						self.zero_image=torch.zeros(img.shape,dtype=torch.float32)
					if self.model!="rgb": self.zero_depth=torch.zeros(depth.shape,dtype=torch.float32)	
			measurements=self.measurements[index][i]
			theta = -float(measurements['robot_pose_yaw'])*np.pi/180.0
			rotMatrix = np.array([[np.cos(theta), -np.sin(theta)], 
							[np.sin(theta),  np.cos(theta)]])

			waypoints = get_waypoints(self.measurements[index], i, self.pred_length, rotMatrix, augment_reflect)

			data['waypoints'].append(torch.tensor(np.array(waypoints)))
			
			#The actions are normalized between [-1,1]
			commands = self.commands[index]
			data['action_acker_speed'].append(torch.tensor(float(commands[i]["desired_speed"])/self.max_speed,dtype=torch.float64))
			#The Unreal Engine Y axis is inverted, so the steering angle is also inverted.
			if not augment_reflect: data['action_acker_steer'].append(torch.tensor(-float(commands[i]["desired_steering_angle"])/self.max_steer,dtype=torch.float64))
			else: data['action_acker_steer'].append(torch.tensor(float(commands[i]["desired_steering_angle"])/self.max_steer,dtype=torch.float64))
			future_action_acker_speed, future_action_acker_steer = [],[]
			for j in range(i+1,min(i+self.pred_length,len(commands))):
				future_action_acker_speed.append(float(commands[j]["desired_speed"])/self.max_speed)
				if not augment_reflect: future_action_acker_steer.append(-float(commands[j]["desired_steering_angle"])/self.max_steer)
				else: future_action_acker_steer.append(float(commands[j]["desired_steering_angle"])/self.max_steer)
			while len(future_action_acker_speed)<self.pred_length-1:
				future_action_acker_speed.append(0.0)
				future_action_acker_steer.append(0.0)
			future_action_acker_speed  = torch.tensor(np.array(future_action_acker_speed))
			future_action_acker_steer = torch.tensor(np.array(future_action_acker_steer))
			data['future_action_acker_speed'].append(future_action_acker_speed)
			data['future_action_acker_steer'].append(future_action_acker_steer)
			vehicle_pose=np.array([float(measurements['robot_pose_x']),float(measurements['robot_pose_y'])])

			#Target point data
			target_point=np.array([float(measurements['target_pos_x']),float(measurements['target_pos_y'])])
			if np.random.rand()<self.prob_localization_disturb:
				target_point+=self.generator.normal(self.mean_error_localization,self.std_error_localization,2)
			local_command_point=np.dot(rotMatrix,target_point)
			if not augment_reflect:local_command_point[1]*=-1 #The Unreal Engine Y axis is inverted.

			data['target_point'].append(torch.tensor(local_command_point))
			#We didn't save the current gear, get if the robot is moving backwards
			gear=1
			if i+1<len(self.measurements[index]):
				next_pose = np.array([float(self.measurements[index][i+1]['robot_pose_x']),
							float(self.measurements[index][i+1]['robot_pose_y'])])
				diference =next_pose-vehicle_pose
				diference =np.dot(rotMatrix,diference)
				if diference[0]<0: gear=-1

			#Ackermann state data
			speed=min(float(measurements['speed'])/self.max_speed,1)
			steer=np.clip(float(measurements['steering_angle'])/self.max_steer,-1,1)
			if np.random.rand()<self.prob_state_disturb:
				speed= np.clip(speed+ self.generator.normal(self.mean_error_state,self.std_error_state),0,1)
				steer=np.clip(steer + self.generator.normal(self.mean_error_state,self.std_error_state),-1,1)
				
			data['speed'].append(torch.tensor(speed*gear,dtype=torch.float32))
			#The Unreal Engine Y axis is inverted, so the steering angle is also inverted.
			if not augment_reflect: data['steer'].append(torch.tensor(-steer,dtype=torch.float32))
			else: data['steer'].append(torch.tensor(steer,dtype=torch.float32))
			
			safety_matrix = self.safety_matrices[index][i]
			safety_matrix = np.array(safety_matrix).astype(np.float32)
			safety_matrix = torch.tensor(safety_matrix,dtype=torch.float32)
			if augment_reflect:
				safety_matrix = safety_matrix.view(self.num_actions_steer,self.num_actions_speed) # To flip correctly 
				data['safety_matrix'].append(torch.flip(safety_matrix, [0]).view(self.num_actions_steer*self.num_actions_speed))
			else:
				data['safety_matrix'].append(safety_matrix)

		for key, value in data.items():
			try:
				data[key]=torch.stack(value)
			except TypeError:
				data[key]=np.array(data[key])
			except Exception as e:
				print("Error in key %s"%key)
				print(e)
				exit()
		return data

def get_waypoints(measurements, id, pred_length, rotMatrix, augment_reflect):
	waypoints = []
	robot_x, robot_y = float(measurements[id]["robot_pose_x"]), float(measurements[id]["robot_pose_y"])
	for i in range(id+1,min(id+pred_length+1,len(measurements))):
		wx, wy = float(measurements[i]["robot_pose_x"]), float(measurements[i]["robot_pose_y"])
		#Transform to local positions
		waypoint = np.dot(rotMatrix,[wx-robot_x, wy-robot_y])
		if not augment_reflect: waypoint[1]*=-1 #The Unreal Engine Y axis is inverted.
		waypoints.append(waypoint)
	while len(waypoints)<pred_length: waypoints.append([0.0,0.0])

	return waypoints

def scale_and_crop_image(image, scale=1, crop_w=256, crop_h=256):
	"""
	Scale and crop a PIL image
	"""
	(width, height) = (int(image.width // scale), int(image.height // scale))
	im_resized = image.resize((width, height))
	start_x = height//2 - crop_h//2
	start_y = width//2 - crop_w//2
	cropped_image = im_resized.crop((start_y, start_x, start_y+crop_w, start_x+crop_h))

	return cropped_image

def scale_and_crop_numpy(array, scale=1, crop_w=256, crop_h=256):
	array = interpolation.zoom(array, 1/scale)
	if array.shape[0]<crop_w or array.shape[1]<crop_h:
		array=np.resize(array,(max(crop_w,array.shape[0]),max(crop_h,array.shape[1])))
	start_x = array.shape[1]//2 - crop_h//2
	start_y = array.shape[0]//2 - crop_w//2
	cropped_array = array[start_y: start_y+crop_w, start_x:start_x+crop_h]

	return cropped_array