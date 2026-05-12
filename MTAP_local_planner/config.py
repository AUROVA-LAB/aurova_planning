import os

class GlobalConfig:
	""" base architecture configurations """
	# Data
	pred_len = 4 # future waypoints predicted

	sequence_size=8

	# data root
	root_dir_all = "/dataset/"

	train_dir=root_dir_all+"train/"
	val_dir=root_dir_all+"val/"
	train_data, val_data = [], []
	if os.path.exists(train_dir):
		train_towns = os.listdir(train_dir)
		val_towns = os.listdir(val_dir)
		for town in train_towns:		
			train_data.append(os.path.join(train_dir, town))
		for town in val_towns:
			val_data.append(os.path.join(val_dir, town))

	ignore_sides = True # don't consider side cameras
	ignore_rear = True # don't consider rear cameras

	input_resolution = [2048,128]

	scale = 1 # image pre-processing
	crop = 256 # image pre-processing

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

	max_throttle = 0.75 # upper limit on throttle signal value in dataset
	brake_speed = 0.4 # desired speed below which brake is triggered
	brake_ratio = 1.1 # ratio of speed to desired speed at which brake is triggered
	clip_delta = 0.25 # maximum change in speed input to logitudinal controller


	aim_dist = 4.0 # distance to search around for aim point
	angle_thresh = 0.3 * max_steer # outlier control detection angle
	dist_thresh = 5 # target point x-distance for outlier filtering
	hidden_size = 1000 #Number of features in the hidden state of the RNN


	speed_weight = 0.05
	semantic_weight = 0.3

	deconv_channel_num_1 = 128 # Number of channels at the first deconvolution layer
	deconv_channel_num_2 = 64 # Number of channels at the second deconvolution layer
	deconv_channel_num_3 = 32 # Number of channels at the third deconvolution layer

	deconv_scale_factor_1 = 8 # Scale factor, of how much the grid size will be interpolated after the first layer
	deconv_scale_factor_2 = 4 # Scale factor, of how much the grid size will be interpolated after the second layer

	num_class = 5
	classes = {
		0: [0, 0, 0],  # unlabeled
		1: [0, 0, 255],  # road
		2: [128, 64, 128],  # sidewalk
		3: [255, 0, 0],  # pedestrian
		4: [0, 255, 0],  # sky
	}
	#Color format BGR
	classes_list = [
		[0, 0, 0],  # unlabeled
		[255, 0, 0],  # road
		[128, 64, 128],  # sidewalk
		[0, 0, 255],  # pedestrian
		[0, 255, 0],  # sky
	]

	pedestrian_class=3

	converter = [
        0,    # unlabeled     =   0u
        1,   # road          =   1
        2,   # sidewalk      =   2
        0,   # bilding      =   3
        0,   # wall          =   4
        0,   # fence         =   5
        0,   # pole          =   6
        0,   # traffic light =   7
        0,   # traffic sign  =   8
        0,   # vegetation    =   9
        2,   # terrain       =  10
        4,   # sky           =  11
        3,   # pedestrian    =  12
        3,   # rider         =  13
        0,   # Car           =  14
        0,   # trck         =  15
        0,   # bs           =  16
        0,   # train         =  17
        0,   # motorcycle    =  18
        0,   # bicycle       =  19
        # custom
        0,   # static        =  20
        0,   # dynamic       =  21
        0,   # other         =  22
        0,   # water         =  23
        1,   # road line     =  24
        2,   # grond        =  25
        2,   # bridge        =  26
        1,   # rail track    =  27
        1    # gard rail    =  28
    ]

	augment = False


	def __init__(self, **kwargs):
		for k,v in kwargs.items():
			setattr(self, k, v)
