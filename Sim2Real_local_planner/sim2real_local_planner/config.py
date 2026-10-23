import os

class GlobalConfig:
	""" base architecture configurations """
	# Data
	pred_len = 4 # future waypoints predicted

	sequence_size=8

	# data root
	root_dir_all = "/dataset/"
	selected_scenes =["ScientificPark_scene1","ScientificPark_scene2","ScientificPark_scene3", "ScientificPark_scene4"]

	train_dir=root_dir_all+"train/"
	val_dir=root_dir_all+"val/"
		

	rgb_path = "rgb_images_octreeGS"
	scale = 2 # Downsampling scale

	input_range_resolution = [2048,128]
	input_rgb_resolution = [1280//scale, 720//scale]

	lr = 1e-4 # learning rate

	# Controller
	turn_KP = 0.75
	turn_KI = 0.75
	turn_KD = 0.3
	turn_n = 40 # buffer size

	speed_KP = 2.0
	speed_KI = 0.5
	speed_KD = 0.5
	speed_n = 20 # buffer size

	max_speed=1.3 #Maximum desired ackermann speed in dataset. The minimum will be -max_speed for backwards.
	max_steer=24.0 #Maximum desired ackermann steering angle in dataset. The minimum will be -max_steer.
	max_target_distance = 35.0

	max_throttle = 0.75 # upper limit on throttle signal value in dataset
	brake_speed = 0.4 # desired speed below which brake is triggered
	brake_ratio = 1.1 # ratio of speed to desired speed at which brake is triggered
	clip_delta = 0.25 # maximum change in speed input to logitudinal controller


	aim_dist = 4.0 # distance to search around for aim point
	angle_thresh = 0.3 * max_steer # outlier control detection angle
	dist_thresh = 5 # target point x-distance for outlier filtering
	hidden_size = 1000 #Number of features in the hidden state of the RNN

	deconv_channel_num = 256 # Number of channels at the deconvolution layer
	num_actions_speed = 9 #Number of speed actions in safety_matrix
	num_actions_steer = 25 #Number of steering actions in safety_matrix

	augment_reflect = True
	prob_color_jitter = 0.5 # probability of color jitter augmentation
	prob_localization_disturb = 0.5 # probability of localization disturbance augmentation
	mean_error_localization = 0.0 # mean of the gaussian noise added to the localization
	std_error_localization = 0.25 # standard deviation of the gaussian noise added to the localization
	prob_state_disturb = 0.5 # probability of ackermann state disturbance augmentation
	mean_error_state = 0.0 # mean of the gaussian noise added to the state
	std_error_state = 0.1 # standard deviation of the gaussian noise added to the state

	def __init__(self, **kwargs):
		for k,v in kwargs.items():
			setattr(self, k, v)
