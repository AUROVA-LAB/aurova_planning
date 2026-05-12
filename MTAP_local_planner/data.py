import os
from pathlib import Path
from PIL import Image
import numpy as np
import torch 
from torch.utils.data import Dataset
from torchvision import transforms as T
from scipy.ndimage import interpolation
from tqdm import tqdm
import sys
import csv

class CARLA_Data(Dataset):

	def __init__(self, root, config, copycat_loss=False):
		self.converter = np.uint8(config.converter)
		self.img_resolution = config.input_resolution
		self.scale = config.scale
		self.augment = config.augment
		self.max_speed=config.max_speed
		self.max_steer=config.max_steer
		self.pred_length = config.pred_len
		self.sequence_size=config.sequence_size
		
		self._im_transform = T.Compose([T.ToTensor(), T.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])])
		# self._depth_transform = T.Compose([T.ToTensor(), T.Normalize(mean=[0.485], std=[0.229])])

		self.images = []
		self.depth_img = []
		self.seg_img = []
		self.measurements = []
		self.commands = []
		self.augment_indices = []
		self.copycat_loss=copycat_loss
		if copycat_loss: self.loss=[]

		self.routes=[]
		data = os.listdir(root)
		for route in data:
			self.routes.append(os.path.join(root, route))

		count=0
		zero_command, self.zero_image, self.zero_image1ch = None, None, None
		for route in tqdm(self.routes, file=sys.stdout):
			
			route_dir = Path(route)

			route_len = len(os.listdir(route_dir / "rgb_images"))
			route_measurements = []
			route_commands = []

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

			if copycat_loss: 
				route_loss=np.load(route_dir / "copycat_loss.npy",allow_pickle=True)
				route_loss=route_loss.astype(float)
			# Loads by sequence
			seq_id=0
			seq_images, seq_depth_img, seq_seg_img = [],[], []
			for idx in range(1,route_len+1):
				count+=1
				seq_images.append(route_dir / "rgb_images" / ("%d.png" % (idx)))
				seq_depth_img.append(route_dir / "depth_images" / ("%d.npy" % (idx)))
				seq_seg_img.append(route_dir / "semantic_images" / ("%d.png" % (idx)))
				seq_id+=1
				if seq_id==self.sequence_size:
					if idx+self.pred_length<=len(route_measurements):
						self.measurements.append(route_measurements[idx-seq_id:idx+self.pred_length])
						self.commands.append(route_commands[idx-seq_id:idx+self.pred_length])
					else:
						self.measurements.append(route_measurements[idx-seq_id:])
						self.commands.append(route_commands[idx-seq_id:])
					if copycat_loss:  self.loss.append(route_loss[idx-seq_id:idx])
					self.images.append(seq_images)
					self.depth_img.append(seq_depth_img)
					self.seg_img.append(seq_seg_img)
					self.augment_indices.append(False)
					if self.augment:
						self.measurements.append(self.measurements[-1])
						self.commands.append(self.commands[-1])
						if copycat_loss: self.loss.append(self.loss[-1])
						self.images.append(seq_images)
						self.depth_img.append(seq_depth_img)
						self.seg_img.append(seq_seg_img)
						self.augment_indices.append(True)
					seq_id=0
					seq_images, seq_depth_img, seq_seg_img = [],[], []
			if seq_id>0:
				if zero_command is None:
					zero_command = {}
					for key in route_commands[0]:
						zero_command[key]=0.0
				last_measurement=route_measurements[-1]
				while seq_id<self.sequence_size:
					route_measurements.append(last_measurement)
					route_commands.append(zero_command)
					if copycat_loss:  route_loss=np.append(route_loss,0.0)
					seq_images.append("None")
					seq_depth_img.append("None")
					seq_seg_img.append("None")
					seq_id += 1
				self.measurements.append(route_measurements[-self.sequence_size:])
				self.commands.append(route_commands[-self.sequence_size:])
				if copycat_loss:  self.loss.append(route_loss[-self.sequence_size:])
				self.images.append(seq_images)
				self.depth_img.append(seq_depth_img)
				self.seg_img.append(seq_seg_img)
				self.augment_indices.append(False)
				#For augmentation, we consider only fliping the data, obtaining the double of sequences. 
				#Indicate to perform the augmentation while loading the sequence.
				if self.augment:
						self.measurements.append(self.measurements[-1])
						self.commands.append(self.commands[-1])
						if copycat_loss:  self.loss.append(self.loss[-1])
						self.images.append(seq_images)
						self.depth_img.append(seq_depth_img)
						self.seg_img.append(seq_seg_img)
						self.augment_indices.append(True)

			
			assert(len(self.measurements)==len(self.commands) and len(self.measurements)==len(self.images))
		
		self.num_sequences=len(self.images)
		if self.augment: count *= 2
		# There is a complex "memory leak"/performance issue when using Python objects like lists in a Dataloader that is loaded with multiprocessing, num_workers > 0
		# A summary of that ongoing discussion can be found here https://github.com/pytorch/pytorch/issues/13246#issuecomment-905703662
		# A workaround is to store the string lists as numpy byte objects because they only have 1 refcount.
		self.images       = np.array(self.images      ).astype(np.string_)
		self.depth_img       = np.array(self.depth_img      ).astype(np.string_)
		self.seg_img       = np.array(self.seg_img      ).astype(np.string_)
		if copycat_loss:  self.loss       = np.array(self.loss)
		print("Loading %d frames from %d folders"%(count, len(self.routes)))
		print("Adding %d empty frames to compleate the sequences"%(self.num_sequences*config.sequence_size-count))
		print("Number of sequences of size %d: %d"%(config.sequence_size,self.num_sequences))


	def __len__(self):
		"""Returns the length of the dataset. """
		return self.num_sequences

	def __getitem__(self, index):
		"""Returns the item at index idx. """
		data = dict()
		augment=self.augment_indices[index]
		for key in ['image','depth','seg','waypoints','action_acker_speed','action_acker_steer',
			  'future_action_acker_speed','future_action_acker_steer','target_point','speed','steer',"image_val"]:
			data[key] = []
		if self.copycat_loss: data["copycat_loss"] = []
		for i in range(self.sequence_size):

			#The None value is converted to string when converting to a np.array
			if self.images[index][i]==b"None":
				data['image'].append(self.zero_image)
				data['image_val'].append(self.zero_image_val)
				data['depth'].append(self.zero_depth)
				data['seg'].append(self.zero_seg)
			else:
				img = Image.open(self.images[index][i])
				if augment: img = img.transpose(Image.FLIP_LEFT_RIGHT)
				img = scale_and_crop_image(img, crop_w=self.img_resolution[0], crop_h=self.img_resolution[1])
				data['image_val'].append(np.array(img.getdata()).reshape(img.size[1], img.size[0], 3))
				img=self._im_transform(img).type(torch.float32)
				data['image'].append(img)

				depth = np.load(self.depth_img[index][i])
				if augment: depth = np.flip(depth, 0)
				depth = scale_and_crop_numpy(depth, crop_w=self.img_resolution[0], crop_h=self.img_resolution[1]).swapaxes(0,1)
				# depth=self._depth_transform(depth).type(torch.float32)
				depth=torch.tensor(depth,dtype=torch.float32)
				data['depth'].append(depth)


				seg = Image.open(self.seg_img[index][i]).convert('L')
				if augment: seg = seg.transpose(Image.FLIP_LEFT_RIGHT)
				seg = scale_and_crop_image(seg, crop_w=self.img_resolution[0], crop_h=self.img_resolution[1])
				seg=torch.tensor(self.converter[seg])
				data['seg'].append(seg)
				if self.zero_image1ch is None:
					self.zero_image=torch.zeros(img.shape,dtype=torch.float32)
					self.zero_depth=torch.zeros(depth.shape,dtype=torch.float32)
					self.zero_seg=torch.zeros(seg.shape,dtype=torch.uint8)
					self.zero_image_val=data['image_val'][0]*0
				
			measurements=self.measurements[index][i]
			theta = -float(measurements['robot_pose_yaw'])*np.pi/180.0
			rotMatrix = np.array([[np.cos(theta), -np.sin(theta)], 
							[np.sin(theta),  np.cos(theta)]])

			waypoints = get_waypoints(self.measurements[index], i, self.pred_length, rotMatrix, augment)

			data['waypoints'].append(torch.tensor(np.array(waypoints)))
			
			#The actions are normalized between [-1,1]
			commands = self.commands[index]
			data['action_acker_speed'].append(torch.tensor(float(commands[i]["desired_speed"])/self.max_speed,dtype=torch.float64))
			if augment: data['action_acker_steer'].append(torch.tensor(-float(commands[i]["desired_steering_angle"])/self.max_steer,dtype=torch.float64))
			else: data['action_acker_steer'].append(torch.tensor(float(commands[i]["desired_steering_angle"])/self.max_steer,dtype=torch.float64))
			future_action_acker_speed, future_action_acker_steer = [],[]
			for j in range(i+1,min(i+self.pred_length+1,len(commands))):
				future_action_acker_speed.append(float(commands[j]["desired_speed"])/self.max_speed)
				if augment: future_action_acker_steer.append(-float(commands[j]["desired_steering_angle"])/self.max_steer)
				else: future_action_acker_steer.append(float(commands[j]["desired_steering_angle"])/self.max_steer)
			while len(future_action_acker_speed)<self.pred_length:
				future_action_acker_speed.append(0.0)
				future_action_acker_steer.append(0.0)
			future_action_acker_speed  = torch.tensor(np.array(future_action_acker_speed))
			future_action_acker_steer = torch.tensor(np.array(future_action_acker_steer))
			data['future_action_acker_speed'].append(future_action_acker_speed)
			data['future_action_acker_steer'].append(future_action_acker_steer)
			vehicle_pose=np.array([float(measurements['robot_pose_x']),float(measurements['robot_pose_y'])])
			target_point=np.array([float(measurements['target_pos_x']),float(measurements['target_pos_y'])])
			local_command_point=np.dot(rotMatrix,target_point)
			if augment:local_command_point[1]*=-1

			data['target_point'].append(torch.tensor(local_command_point))
			#We didn't save the current gear, get if the robot is moving backwards
			gear=1
			if i+1<len(self.measurements[index]):
				next_pose = np.array([float(self.measurements[index][i+1]['robot_pose_x']),
							float(self.measurements[index][i+1]['robot_pose_y'])])
				diference =next_pose-vehicle_pose
				diference =np.dot(rotMatrix,diference)
				if diference[0]<0: gear=-1
			data['speed'].append(torch.tensor(min(float(measurements['speed'])/self.max_speed,1)*gear,dtype=torch.float32))
			if augment: data['steer'].append(torch.tensor(-float(measurements['steering_angle'])/self.max_steer,dtype=torch.float32))
			else: data['steer'].append(torch.tensor(float(measurements['steering_angle'])/self.max_steer,dtype=torch.float32))
			if self.copycat_loss: data["copycat_loss"].append(torch.tensor(self.loss[index][i],dtype=torch.float32))
		for key, value in data.items():
			try:
				data[key]=torch.stack(value)
			except TypeError:
				data[key]=np.array(data[key])
		return data

def get_waypoints(measurements, id, pred_length, rotMatrix, augment):
	waypoints = []
	robot_x, robot_y = float(measurements[id]["robot_pose_x"]), float(measurements[id]["robot_pose_y"])
	for i in range(id+1,min(id+pred_length+1,len(measurements))):
		wx, wy = float(measurements[i]["robot_pose_x"]), float(measurements[i]["robot_pose_y"])
		#Transform to local positions
		waypoint = np.dot(rotMatrix,[wx-robot_x, wy-robot_y])
		if augment: waypoint[1]*=-1
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
	array_resized = interpolation.zoom(array, scale)
	start_x = array.shape[1]//2 - crop_h//2
	start_y = array.shape[0]//2 - crop_w//2
	cropped_array = array_resized[start_y: start_y+crop_w, start_x:start_x+crop_h]

	return cropped_array


# A reduce dataset with only the actions for the copycat model.
class Action_Data(Dataset):

	def __init__(self, root, config):
		self.max_speed=config.max_speed
		self.max_steer=config.max_steer
		self.pred_length = config.pred_len
		self.sequence_size=config.sequence_size

		self.measurements = []
		self.commands = []
		self.output_dir = []

		self.routes=[]
		data = os.listdir(root)
		for route in data:
			self.routes.append(os.path.join(root, route))

		for route in tqdm(self.routes, file=sys.stdout):
			route_dir = Path(route)

			route_measurements = []
			route_commands = []

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


			self.measurements.append(route_measurements)
			self.commands.append(route_commands)
			self.output_dir.append(str(route_dir)+"/copycat_loss.npy")
				
		print("Loading %d folders"%(len(self.routes)))


	def __len__(self):
		"""Returns the length of the dataset. """
		return len(self.measurements)

	def __getitem__(self, index):

		"""Returns one route. """
		return route_data(self.measurements[index],self.commands[index],self.output_dir[index],self)

def get_waypoints(measurements, id, pred_length, rotMatrix, augment=False):
	waypoints = []
	robot_x, robot_y = float(measurements[id]["robot_pose_x"]), float(measurements[id]["robot_pose_y"])
	for i in range(id+1,min(id+pred_length+1,len(measurements))):
		wx, wy = float(measurements[i]["robot_pose_x"]), float(measurements[i]["robot_pose_y"])
		#Transform to local positions
		waypoint = np.dot(rotMatrix,[wx-robot_x, wy-robot_y])
		if augment: waypoint[1]*=-1
		waypoints.append(waypoint)
	while len(waypoints)<pred_length: waypoints.append([0.0,0.0])
	return waypoints


class route_data(Dataset):

	def __init__(self,measurements, commands, output_dir, parent: Action_Data):
		self.measurements=measurements
		self.commands=commands
		self.output_dir=output_dir
		self.parent=parent

	def __len__(self):
		return len(self.measurements)
	
	def __getitem__(self, index):
		data=dict()
		if index>0:
			data["speed"]=torch.tensor(float(self.commands[index-1]["desired_speed"])/self.parent.max_speed,dtype=torch.float32)
			data["steer"]=torch.tensor(float(self.commands[index-1]["desired_speed"])/self.parent.max_speed,dtype=torch.float32)
		else:
			data["speed"]=torch.tensor(0.0,dtype=torch.float32)
			data["steer"]=torch.tensor(0.0,dtype=torch.float32)
		theta = -float(self.measurements[index]['robot_pose_yaw'])*np.pi/180.0
		rotMatrix = np.array([[np.cos(theta), -np.sin(theta)], 
						[np.sin(theta),  np.cos(theta)]])
		
		waypoints = get_waypoints(self.measurements, index, self.parent.pred_length, rotMatrix)

		data['waypoints']=torch.tensor(np.array(waypoints))
			
		#The actions are normalized between [-1,1]
		data['action_acker_speed']=torch.tensor(float(self.commands[index]["desired_speed"])/self.parent.max_speed,dtype=torch.float32)
		data['action_acker_steer']=torch.tensor(float(self.commands[index]["desired_steering_angle"])/self.parent.max_steer,dtype=torch.float32)
		future_action_acker_speed, future_action_acker_steer = [],[]
		for j in range(index+1,min(index+self.parent.pred_length+1,len(self.commands))):
			future_action_acker_speed.append(float(self.commands[j]["desired_speed"])/self.parent.max_speed)
			future_action_acker_steer.append(float(self.commands[j]["desired_steering_angle"])/self.parent.max_steer)
		while len(future_action_acker_speed)<self.parent.pred_length:
			future_action_acker_speed.append(0.0)
			future_action_acker_steer.append(0.0)
		data['future_action_acker_speed'] = torch.tensor(np.array(future_action_acker_speed),dtype=torch.float32)
		data['future_action_acker_steer']= torch.tensor(np.array(future_action_acker_steer),dtype=torch.float32)
		return data