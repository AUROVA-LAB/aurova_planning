from collections import deque
import numpy as np
import torch 
from torch import nn
from resnet import *


class PIDController(object):
	def __init__(self, K_P=1.0, K_I=0.0, K_D=0.0, n=20):
		self._K_P = K_P
		self._K_I = K_I
		self._K_D = K_D

		self._window = deque([0 for _ in range(n)], maxlen=n)
		self._max = 0.0
		self._min = 0.0

	def step(self, error):
		self._window.append(error)
		self._max = max(self._max, abs(error))
		self._min = -abs(self._max)

		if len(self._window) >= 2:
			integral = np.mean(self._window)
			derivative = (self._window[-1] - self._window[-2])
		else:
			integral = 0.0
			derivative = 0.0

		return self._K_P * error + self._K_I * integral + self._K_D * derivative
	
class SegDecoder(nn.Module):
    def __init__(self, config, latent_dim=512):
        super().__init__()
        self.config = config
        self.latent_dim = latent_dim
        self.num_class = config.num_class

        self.deconv1 = nn.Sequential(
                    nn.Conv2d(self.latent_dim, self.config.deconv_channel_num_1, 3, 1, 1),
                    nn.ReLU(True),
                    nn.Conv2d(self.config.deconv_channel_num_1, self.config.deconv_channel_num_2, 3, 1, 1),
                    nn.ReLU(True),
                    )
        self.deconv2 = nn.Sequential(
                    nn.Conv2d(self.config.deconv_channel_num_2, self.config.deconv_channel_num_3, 3, 1, 1),
                    nn.ReLU(True),
                    nn.Conv2d(self.config.deconv_channel_num_3, self.config.deconv_channel_num_3, 3, 1, 1),
                    nn.ReLU(True),
                    )
        self.deconv3 = nn.Sequential(
                    nn.Conv2d(self.config.deconv_channel_num_3, self.config.deconv_channel_num_3, 3, 1, 1),
                    nn.ReLU(True),
                    nn.Conv2d(self.config.deconv_channel_num_3, self.num_class, 3, 1, 1),
					nn.Sigmoid()
                    )

    def forward(self, x):
        x = self.deconv1(x)
        x = nn.functional.interpolate(x, scale_factor=self.config.deconv_scale_factor_1, mode='bilinear', align_corners=False)
        x = self.deconv2(x)
        x = nn.functional.interpolate(x, scale_factor=self.config.deconv_scale_factor_2, mode='bilinear', align_corners=False)
        x = self.deconv3(x)

        return x

class MTAP_Network(nn.Module):

	def __init__(self, config):
		super().__init__()
		self.config = config
		self.device='cpu' #Init device, use to function to change device

		self.speed_controller = PIDController(K_P=config.speed_KP, K_I=config.speed_KI, K_D=config.speed_KD, n=config.speed_n)

		self.perception = resnet34(pretrained=True)

		#Recurrent layer
		self.rnn = nn.LSTMCell(1000,self.config.hidden_size)
		#Init hidden and cell output for batch_size=1
		self.hidden = nn.Parameter(torch.zeros([1,self.config.hidden_size]))
		self.cell = nn.Parameter(torch.zeros([1,self.config.hidden_size]))

		self.seg_decoder=SegDecoder(self.config)

		self.measurements = nn.Sequential(
							nn.Linear(2+2, 128),
							nn.ReLU(inplace=True),
							nn.Linear(128, 128),
							nn.ReLU(inplace=True),
						)

		self.join_traj = nn.Sequential(
							nn.Linear(128+1000, 512),
							nn.ReLU(inplace=True),
							nn.Linear(512, 512),
							nn.ReLU(inplace=True),
							nn.Linear(512, 256),
							nn.ReLU(inplace=True),
						)

		self.join_ctrl = nn.Sequential(
							nn.Linear(128+512+1000, 512),
							nn.ReLU(inplace=True),
							nn.Linear(512, 512),
							nn.ReLU(inplace=True),
							nn.Linear(512, 256),
							nn.ReLU(inplace=True),
						)

		self.policy_head = nn.Sequential(
				nn.Linear(256, 256),
				nn.ReLU(inplace=True),
				nn.Linear(256, 256),
				nn.Dropout(p=0.5),
				nn.ReLU(inplace=True),
			)
		self.decoder_ctrl = nn.GRUCell(input_size=256+2, hidden_size=256)
		self.output_ctrl = nn.Sequential(
				nn.Linear(256, 256),
				nn.ReLU(inplace=True),
				nn.Linear(256, 256),
				nn.ReLU(inplace=True),
			)

		# Action command exit in range [-1,1]
		self.dist_acker_speed = nn.Sequential(nn.Linear(256, 1), nn.Tanh())
		self.dist_acker_steer = nn.Sequential(nn.Linear(256, 1), nn.Tanh())


		self.decoder_traj = nn.GRUCell(input_size=4, hidden_size=256)
		self.output_traj = nn.Linear(256, 2)

		self.init_att = nn.Sequential(
				nn.Linear(128, 256),
				nn.ReLU(inplace=True),
				nn.Linear(256, 256),
				nn.Softmax(1)
			)

		self.wp_att = nn.Sequential(
				nn.Linear(256+256, 256),
				nn.ReLU(inplace=True),
				nn.Linear(256, 256),
				nn.Softmax(1)
			)

		self.merge = nn.Sequential(
				nn.Linear(512+256, 512),
				nn.ReLU(inplace=True),
				nn.Linear(512, 256),
			)
		
	def reset_RNN(self,batch_size=1):
		self.hidden = nn.Parameter(torch.zeros(batch_size, self.config.hidden_size).to(self.device))
		self.cell = nn.Parameter(torch.zeros(batch_size, self.config.hidden_size).to(self.device))

	def set_RNN_state(self, hidden, cell):
		self.hidden = hidden
		self.cell = cell
	
	def get_RNN_state(self):
		return self.hidden, self.cell

	def to(self, device):
		super().to(device)
		self.device = device

		
	def forward(self, img, state, target_point):
		outputs=dict()
		for key in ['seg_img','pred_wp','acker_speed_branches','acker_steer_branches','future_acker_speed','future_acker_steer']:
			outputs[key]=[]
		shape=state.shape[:2]
		self.reset_RNN(shape[0])
		for i in range(shape[1]):
			feature_emb, cnn_feature = self.perception(img[:,i])
			feature_emb, _ = self.rnn(feature_emb,(self.hidden, self.cell))
			outputs['seg_img'].append(self.seg_decoder(cnn_feature))
			measurement_feature = self.measurements(state[:,i])
			
			j_traj = self.join_traj(torch.cat([feature_emb, measurement_feature], 1))

			z = j_traj
			output_wp = list()
			traj_hidden_state = list()

			# initial input variable to GRU
			x = torch.zeros(size=(z.shape[0], 2), dtype=z.dtype).type_as(z)

			# autoregressive generation of output waypoints
			for _ in range(self.config.pred_len):
				x_in = torch.cat([x, target_point[:,i]], dim=1)
				z = self.decoder_traj(x_in, z)
				traj_hidden_state.append(z)
				dx = self.output_traj(z)
				x = dx + x
				output_wp.append(x)

			pred_wp = torch.stack(output_wp, dim=1)
			outputs['pred_wp'].append(pred_wp)

			traj_hidden_state = torch.stack(traj_hidden_state, dim=1)
			init_att = self.init_att(measurement_feature).view(-1, 1, 4, 64)
			init_att = torch.sum(cnn_feature*init_att, dim=(2, 3))
			j_ctrl = self.join_ctrl(torch.cat([feature_emb, measurement_feature, init_att], 1))
	
			policy = self.policy_head(j_ctrl)
			acker_speed=self.dist_acker_speed(policy)
			acker_steer=self.dist_acker_steer(policy)
			outputs['acker_speed_branches'].append(acker_speed)
			outputs['acker_steer_branches'].append(acker_steer)

			x = j_ctrl
			future_acker_speed, future_acker_steer = [], []

			# initial hidden variable to GRU
			h = torch.zeros(size=(x.shape[0], 256), dtype=x.dtype).type_as(x)

			for _ in range(self.config.pred_len):
				x_in = torch.cat([x, acker_speed, acker_steer], dim=1)
				h = self.decoder_ctrl(x_in, h)
				wp_att = self.wp_att(torch.cat([h, traj_hidden_state[:, _]], 1)).view(-1, 1, 4, 64)
				new_feature_emb = torch.sum(cnn_feature*wp_att, dim=(2, 3))
				merged_feature = self.merge(torch.cat([h, new_feature_emb], 1))
				dx = self.output_ctrl(merged_feature)
				x = dx + x

				policy = self.policy_head(x)
				acker_speed = self.dist_acker_speed(policy)
				acker_steer = self.dist_acker_steer(policy)
				future_acker_speed.append(acker_speed)
				future_acker_steer.append(acker_steer)


			outputs['future_acker_speed'].append(torch.stack(future_acker_speed, dim=1))
			outputs['future_acker_steer'].append(torch.stack(future_acker_steer, dim=1))
		
		for key, value in outputs.items():
			outputs[key]=torch.stack(value,dim=1)
		return outputs
	
	def forward_inference(self, img, state, target_point):
		feature_emb, cnn_feature = self.perception(img)
		feature_emb, _ = self.rnn(feature_emb,(self.hidden, self.cell))
		outputs = {}
		outputs['seg_img'] = self.seg_decoder(cnn_feature)
		measurement_feature = self.measurements(state)
		
		j_traj = self.join_traj(torch.cat([feature_emb, measurement_feature], 1))
		z = j_traj
		output_wp = list()
		traj_hidden_state = list()

		# initial input variable to GRU
		x = torch.zeros(size=(z.shape[0], 2), dtype=z.dtype).type_as(z)

		# autoregressive generation of output waypoints
		for _ in range(self.config.pred_len):
			x_in = torch.cat([x, target_point], dim=1)
			z = self.decoder_traj(x_in, z)
			traj_hidden_state.append(z)
			dx = self.output_traj(z)
			x = dx + x
			output_wp.append(x)

		pred_wp = torch.stack(output_wp, dim=1)
		outputs['pred_wp'] = pred_wp

		traj_hidden_state = torch.stack(traj_hidden_state, dim=1)
		init_att = self.init_att(measurement_feature).view(-1, 1, 4, 64)
		init_att = torch.sum(cnn_feature*init_att, dim=(2, 3))
		j_ctrl = self.join_ctrl(torch.cat([feature_emb, measurement_feature, init_att], 1))
		policy = self.policy_head(j_ctrl)
		outputs['acker_speed_branches'] = self.dist_acker_speed(policy)
		outputs['acker_steer_branches'] = self.dist_acker_steer(policy)

		x = j_ctrl
		acker_speed = outputs['acker_speed_branches']
		acker_steer = outputs['acker_steer_branches']
		future_acker_speed, future_acker_steer = [], []

		h = torch.zeros(size=(x.shape[0], 256), dtype=x.dtype).type_as(x)

		for _ in range(self.config.pred_len):
			x_in = torch.cat([x, acker_speed, acker_steer], dim=1)
			h = self.decoder_ctrl(x_in, h)
			wp_att = self.wp_att(torch.cat([h, traj_hidden_state[:, _]], 1)).view(-1, 1, 4, 64)
			new_feature_emb = torch.sum(cnn_feature*wp_att, dim=(2, 3))
			merged_feature = self.merge(torch.cat([h, new_feature_emb], 1))
			dx = self.output_ctrl(merged_feature)
			x = dx + x

			policy = self.policy_head(x)
			acker_speed = self.dist_acker_speed(policy)
			acker_steer = self.dist_acker_steer(policy)
			future_acker_speed.append(acker_speed)
			future_acker_steer.append(acker_steer)


		outputs['future_acker_speed'] = torch.stack(future_acker_speed, dim=1)
		outputs['future_acker_steer'] = torch.stack(future_acker_steer, dim=1)
		return outputs
	
	def control_pid(self, waypoints, target):
		''' Predicts vehicle control with a PID controller.
		Args:
			waypoints (tensor): output of self.plan()
			velocity (tensor): speedometer input
		'''

		DIFFERENT_ANGLE_THRESHOLD=np.pi/4
		ENOUGH_DISTANCE=0.05
		#The data was get at 10FPS, we get the speed at m/s.
		fps=10
		target=waypoints[0]
		angle = np.arctan2(target[1],target[0])
		id=1
		for i in range(1,len(waypoints)):
			distance=np.linalg.norm(waypoints[i])
			angle2=np.arctan2(waypoints[i][1]-target[1],waypoints[i][0]-target[0])
			if distance>ENOUGH_DISTANCE and abs(angle2-angle)>=DIFFERENT_ANGLE_THRESHOLD:
				break
			
			angle = angle2
			target=waypoints[i]
			id+=1
		
		steering = np.arctan2(target[1],target[0])
		if steering>np.pi/2: steering=np.pi - steering
		elif steering<-np.pi/2: steering=-np.pi - steering
		speed = target[0]/(np.cos(steering)*(id/fps))
		steering*=-360.0/np.pi #To degrees, and positive angles are negative in the control.
		return speed, steering
	

class MTAP_Network_without_Semantic(nn.Module):

	def __init__(self, config):
		super().__init__()
		self.config = config
		self.device='cpu' #Init device, use to function to change device

		self.speed_controller = PIDController(K_P=config.speed_KP, K_I=config.speed_KI, K_D=config.speed_KD, n=config.speed_n)

		self.perception = resnet34(pretrained=True)

		#Recurrent layer
		self.rnn = nn.LSTMCell(1000,self.config.hidden_size)
		#Init hidden and cell output for batch_size=1
		self.hidden = nn.Parameter(torch.zeros([1,self.config.hidden_size]))
		self.cell = nn.Parameter(torch.zeros([1,self.config.hidden_size]))

		self.measurements = nn.Sequential(
							nn.Linear(2+2, 128),
							nn.ReLU(inplace=True),
							nn.Linear(128, 128),
							nn.ReLU(inplace=True),
						)

		self.join_traj = nn.Sequential(
							nn.Linear(128+1000, 512),
							nn.ReLU(inplace=True),
							nn.Linear(512, 512),
							nn.ReLU(inplace=True),
							nn.Linear(512, 256),
							nn.ReLU(inplace=True),
						)

		self.join_ctrl = nn.Sequential(
							nn.Linear(128+512+1000, 512),
							nn.ReLU(inplace=True),
							nn.Linear(512, 512),
							nn.ReLU(inplace=True),
							nn.Linear(512, 256),
							nn.ReLU(inplace=True),
						)
		# shared branches_neurons

		self.policy_head = nn.Sequential(
				nn.Linear(256, 256),
				nn.ReLU(inplace=True),
				nn.Linear(256, 256),
				nn.Dropout(p=0.5),
				nn.ReLU(inplace=True),
			)
		self.decoder_ctrl = nn.GRUCell(input_size=256+2, hidden_size=256)
		self.output_ctrl = nn.Sequential(
				nn.Linear(256, 256),
				nn.ReLU(inplace=True),
				nn.Linear(256, 256),
				nn.ReLU(inplace=True),
			)
		
		self.dist_acker_speed = nn.Sequential(nn.Linear(256, 1), nn.Tanh())
		self.dist_acker_steer = nn.Sequential(nn.Linear(256, 1), nn.Tanh())


		self.decoder_traj = nn.GRUCell(input_size=4, hidden_size=256)
		self.output_traj = nn.Linear(256, 2)

		self.init_att = nn.Sequential(
				nn.Linear(128, 256),
				nn.ReLU(inplace=True),
				nn.Linear(256, 256),
				nn.Softmax(1)
			)

		self.wp_att = nn.Sequential(
				nn.Linear(256+256, 256),
				nn.ReLU(inplace=True),
				nn.Linear(256, 256),
				nn.Softmax(1)
			)

		self.merge = nn.Sequential(
				nn.Linear(512+256, 512),
				nn.ReLU(inplace=True),
				nn.Linear(512, 256),
			)
		
	def reset_RNN(self,batch_size=1):
		self.hidden = nn.Parameter(torch.zeros(batch_size, self.config.hidden_size).to(self.device))
		self.cell = nn.Parameter(torch.zeros(batch_size, self.config.hidden_size).to(self.device))

	def set_RNN_state(self, hidden, cell):
		self.hidden = hidden
		self.cell = cell
	
	def get_RNN_state(self):
		return self.hidden, self.cell

	def to(self, device):
		super().to(device)
		self.device = device

		
	def forward(self, img, state, target_point):
		outputs=dict()
		for key in ['pred_wp','acker_speed_branches','acker_steer_branches','future_acker_speed','future_acker_steer']:
			outputs[key]=[]
		shape=state.shape[:2]
		self.reset_RNN(shape[0])
		for i in range(shape[1]):
			feature_emb, cnn_feature = self.perception(img[:,i])
			feature_emb, _ = self.rnn(feature_emb,(self.hidden, self.cell))
			measurement_feature = self.measurements(state[:,i])
			
			j_traj = self.join_traj(torch.cat([feature_emb, measurement_feature], 1))
			z = j_traj
			output_wp = list()
			traj_hidden_state = list()

			# initial input variable to GRU
			x = torch.zeros(size=(z.shape[0], 2), dtype=z.dtype).type_as(z)

			# autoregressive generation of output waypoints
			for _ in range(self.config.pred_len):
				x_in = torch.cat([x, target_point[:,i]], dim=1)
				z = self.decoder_traj(x_in, z)
				traj_hidden_state.append(z)
				dx = self.output_traj(z)
				x = dx + x
				output_wp.append(x)

			pred_wp = torch.stack(output_wp, dim=1)
			outputs['pred_wp'].append(pred_wp)

			traj_hidden_state = torch.stack(traj_hidden_state, dim=1)
			init_att = self.init_att(measurement_feature).view(-1, 1, 4, 64)
			init_att = torch.sum(cnn_feature*init_att, dim=(2, 3))
			j_ctrl = self.join_ctrl(torch.cat([feature_emb, measurement_feature, init_att], 1))
			policy = self.policy_head(j_ctrl)
			acker_speed=self.dist_acker_speed(policy)
			acker_steer=self.dist_acker_steer(policy)
			outputs['acker_speed_branches'].append(acker_speed)
			outputs['acker_steer_branches'].append(acker_steer)

			x = j_ctrl
			future_acker_speed, future_acker_steer = [], []

			# initial hidden variable to GRU
			h = torch.zeros(size=(x.shape[0], 256), dtype=x.dtype).type_as(x)

			for _ in range(self.config.pred_len):
				x_in = torch.cat([x, acker_speed, acker_steer], dim=1)
				h = self.decoder_ctrl(x_in, h)
				wp_att = self.wp_att(torch.cat([h, traj_hidden_state[:, _]], 1)).view(-1, 1, 4, 64)
				new_feature_emb = torch.sum(cnn_feature*wp_att, dim=(2, 3))
				merged_feature = self.merge(torch.cat([h, new_feature_emb], 1))
				dx = self.output_ctrl(merged_feature)
				x = dx + x

				policy = self.policy_head(x)
				acker_speed = self.dist_acker_speed(policy)
				acker_steer = self.dist_acker_steer(policy)
				future_acker_speed.append(acker_speed)
				future_acker_steer.append(acker_steer)


			outputs['future_acker_speed'].append(torch.stack(future_acker_speed, dim=1))
			outputs['future_acker_steer'].append(torch.stack(future_acker_steer, dim=1))
		
		for key, value in outputs.items():
			outputs[key]=torch.stack(value,dim=1)
		return outputs
	
	def forward_inference(self, img, state, target_point):
		feature_emb, cnn_feature = self.perception(img)
		feature_emb, _ = self.rnn(feature_emb,(self.hidden, self.cell))
		outputs = {}
		measurement_feature = self.measurements(state)
		
		j_traj = self.join_traj(torch.cat([feature_emb, measurement_feature], 1))
		z = j_traj
		output_wp = list()
		traj_hidden_state = list()

		# initial input variable to GRU
		x = torch.zeros(size=(z.shape[0], 2), dtype=z.dtype).type_as(z)

		# autoregressive generation of output waypoints
		for _ in range(self.config.pred_len):
			x_in = torch.cat([x, target_point], dim=1)
			z = self.decoder_traj(x_in, z)
			traj_hidden_state.append(z)
			dx = self.output_traj(z)
			x = dx + x
			output_wp.append(x)

		pred_wp = torch.stack(output_wp, dim=1)
		outputs['pred_wp'] = pred_wp

		traj_hidden_state = torch.stack(traj_hidden_state, dim=1)
		init_att = self.init_att(measurement_feature).view(-1, 1, 4, 64)
		init_att = torch.sum(cnn_feature*init_att, dim=(2, 3))
		j_ctrl = self.join_ctrl(torch.cat([feature_emb, measurement_feature, init_att], 1))
		policy = self.policy_head(j_ctrl)
		outputs['acker_speed_branches'] = self.dist_acker_speed(policy)
		outputs['acker_steer_branches'] = self.dist_acker_steer(policy)

		x = j_ctrl
		acker_speed = outputs['acker_speed_branches']
		acker_steer = outputs['acker_steer_branches']
		future_acker_speed, future_acker_steer = [], []

		# initial hidden variable to GRU
		h = torch.zeros(size=(x.shape[0], 256), dtype=x.dtype).type_as(x)

		for _ in range(self.config.pred_len):
			x_in = torch.cat([x, acker_speed, acker_steer], dim=1)
			h = self.decoder_ctrl(x_in, h)
			wp_att = self.wp_att(torch.cat([h, traj_hidden_state[:, _]], 1)).view(-1, 1, 4, 64)
			new_feature_emb = torch.sum(cnn_feature*wp_att, dim=(2, 3))
			merged_feature = self.merge(torch.cat([h, new_feature_emb], 1))
			dx = self.output_ctrl(merged_feature)
			x = dx + x

			policy = self.policy_head(x)
			acker_speed = self.dist_acker_speed(policy)
			acker_steer = self.dist_acker_steer(policy)
			future_acker_speed.append(acker_speed)
			future_acker_steer.append(acker_steer)


		outputs['future_acker_speed'] = torch.stack(future_acker_speed, dim=1)
		outputs['future_acker_steer'] = torch.stack(future_acker_steer, dim=1)
		return outputs

	def control_pid(self, waypoints, target):
		''' Predicts vehicle control with a PID controller.
		Args:
			waypoints (tensor): output of self.plan()
			velocity (tensor): speedometer input
		'''

		DIFFERENT_ANGLE_THRESHOLD=np.pi/4
		ENOUGH_DISTANCE=0.05
		#The data was get at 10FPS, we get the speed at m/s.
		fps=10
		target=waypoints[0]
		angle = np.arctan2(target[1],target[0])
		id=1
		for i in range(1,len(waypoints)):
			distance=np.linalg.norm(waypoints[i])
			angle2=np.arctan2(waypoints[i][1]-target[1],waypoints[i][0]-target[0])
			if distance>ENOUGH_DISTANCE and abs(angle2-angle)>=DIFFERENT_ANGLE_THRESHOLD:
				break
			#Positive aim angles need a negative control steering.
			angle = angle2
			target=waypoints[i]
			id+=1
		
		steering = np.arctan2(target[1],target[0])
		if steering>np.pi/2: steering=np.pi - steering
		elif steering<-np.pi/2: steering=-np.pi - steering
		speed = target[0]/(np.cos(steering)*(id/fps))
		steering*=-360.0/np.pi #To degrees, and positive angles are negative in the control.
		return speed, steering