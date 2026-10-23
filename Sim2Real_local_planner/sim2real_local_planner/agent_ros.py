from __future__ import print_function
import os, sys
import json
from copy import deepcopy

from PIL import Image
from collections import deque

sys.path.append("/catkin_ws/src/Sim2Real")
import torch
import numpy as np
import math
import time
from sim2real_local_planner.model import *
from sim2real_local_planner.config import GlobalConfig
from torchvision import transforms as T
from scipy.spatial.transform import Rotation as R

import itertools
import pathlib
import argparse

SAVE_PATH = os.environ.get('SAVE_PATH')

if not SAVE_PATH:
    SAVE_PATH = None
else:
    pathlib.Path(SAVE_PATH).mkdir(parents=True, exist_ok=True)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image 
from geometry_msgs.msg import PoseWithCovarianceStamped
from pyquaternion import Quaternion 
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import MarkerArray, Marker
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import cv2

NEAR_COLLISION_DISTANCE = [2.5, 2.7] # meters
NEAR_COLLISION_DISTANCE2 = [1.2, 1.4] # Stop if obstacle is nearest than this distance from the sensor
MINIMUM_DISTANCE = 2.0
MAX_TIME_STOPPED = 5.0 #Stopped or local minimum


class Agent(Node):
    def __init__(self):
        super().__init__('agent_ros')

        self.init=False
        #Subscribers
        self.image_sub = Subscriber(self, Image,"/rgb_image")
        self.depth_sub = Subscriber(self, Image, "/range_image")
        self.pose_sub = Subscriber(self, Odometry, "/pose")
        self.state_sub = self.create_subscription( AckermannDrive,"/ackermann_state", self.state_callback,10)
        self.goal_sub = self.create_subscription( PoseWithCovarianceStamped,"/goal", self.goal_callback,10)
        

        self.sync_subs = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.pose_sub], queue_size=10, slop=0.5)
        self.sync_subs.registerCallback(self.callback)

        #CV bridge
        self.bridge = CvBridge()

        #Publishers
        self.drive_rad_pub = self.create_publisher( AckermannDrive,"/ackermann_cmd", 1)
        self.drive_deg_pub = self.create_publisher( AckermannDrive,"desired_ackermann_state", 1)
        self.forw_rec_pub = self.create_publisher( Float32,"/forward_recommended_velocity", 1)
        self.back_rec_pub = self.create_publisher( Float32,"/backward_recommended_velocity", 1)
        self.marker_pub = self.create_publisher( MarkerArray,"/ackermann_vis", 1)

        #Agent parameters
        self.status = 0
        self.last_steers=deque()
        self._im_transform = T.Compose([T.ToTensor(), T.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])])
        self._depth_transform = T.Compose([T.ToTensor(), T.Normalize(mean=[0.485], std=[0.229])])
        self.config = GlobalConfig()
        self.time_local_minimum=0
        self.previous_pose = [0,0]
        self.steer_local_minimum = 1.0 #Target steer for trying to exit from a local minimum. It alternates between minimum and maximum steering angle.
        self.safe_stop=False
        self.declare_parameter("model","rgb+depth")
        self.declare_parameter("inverted_axis",False)
        self.declare_parameter("model_path","")
        self.model = self.get_parameter("model").value
        self.inverted_axis = self.get_parameter("inverted_axis").value #The Y axis in Unreal Engine is inverted, so the steering angle is also inverted. (For old models)
        if self.inverted_axis:
            print("Inverted axis enabled, steering angle will be inverted.", flush=True)
        self.tick_data = None
        self.goal, self.state = None, None
        self.drive_message=AckermannDrive()
        self.marker_message=MarkerArray()

        # Load model file 
        state_dict = torch.load(self.get_parameter("model_path").value)
        #Remove the parameters cell and hidden
        try:
            del state_dict["hidden"], state_dict["cell"]
        except: 
            pass

        if self.model == "rgb": self.net = RGB_Network(self.config, False)
        elif self.model == "depth": self.net = Depth_Network(self.config, False)
        elif self.model == "rgb+depth": self.net = MTAP_Network(self.config, False)
        else: raise ValueError("Model not supported")
        
        self.net.to("cuda")
        self.net.load_state_dict(state_dict, strict = False)
        self.net.cuda()
        self.net.eval()
        
        self.init=True

    def callback(self,image:Image,depth:Image,pose:Odometry):
        #Read the messages
        if self.goal is None or self.state is None or self.init==False: return
        state=self.state
        speed=state.speed/self.config.max_speed
        steer=np.clip(state.steering_angle/self.config.max_steer,-1,1)
        clock=pose.header.stamp.nanosec*1e-9+pose.header.stamp.sec
        position = np.array([pose.pose.pose.position.x, pose.pose.pose.position.y,0]) 
        # Use q.w to get the rotation from global to local.
        quat = Quaternion(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, -pose.pose.pose.orientation.z)
        rotation_matrix = quat.rotation_matrix
        
        target_point=self.goal-position
        target_point = np.dot(rotation_matrix, target_point)
        target_point = target_point[0:2]
        if self.inverted_axis:
            target_point[1] *= -1
            steer *= -1 #The steering angle is also inverted, so the steer is inverted.
        # Clip the target point in range of the training data.
        distance= np.linalg.norm(target_point)
        if distance > self.config.max_target_distance:
            target_point = target_point * np.sqrt(self.config.max_target_distance / distance)
        tick_data = {
                'speed': speed,
                'steer': steer,
                'target_point':target_point,
                'clock': clock,
                'position': position,
                'rotation': rotation_matrix.T, #The rotation matrix is used afterwards to pass from local to global, so the inverse is used.
                }
        
        if self.model != "depth":
            tick_data["rgb"] = self.bridge.imgmsg_to_cv2(image, "rgb8")
        if self.model != "rgb":
            ranges = self.bridge.imgmsg_to_cv2(depth, "mono16")
            max_range = 50.0
            np.place(ranges, ranges == 0, 65535) #Replace no detections with maximum value
            ranges = ranges * 261.0 / pow(2,16) #Convert to meters
            ranges = np.clip(ranges, 0, max_range)
            ranges = max_range - ranges
            ranges = (ranges / max_range * 65535).astype(np.uint16)
            tick_data["depth"]  = ranges
        self.tick_data = tick_data
        self.inference()

    @torch.inference_mode() # Faster version of torch_no_grad
    def inference(self):
        # prepare image input
        image, depth = self.prepare_image(self.tick_data)

        # prepare state and goal input
        target_point = torch.tensor(self.tick_data['target_point']).to('cuda', dtype=torch.float32)
        speed = torch.tensor(self.tick_data['speed']).to('cuda',dtype=torch.float32).view(1)
        steer = torch.tensor(self.tick_data['steer']).to('cuda',dtype=torch.float32).view(1)
        state = torch.cat([speed, steer, target_point]).unsqueeze(0)
        # forward pass
        with torch.no_grad():
            if self.model == "rgb": pred = self.net.forward_inference(image, state, target_point.unsqueeze(0))
            elif self.model == "depth": pred = self.net.forward_inference(depth, state, target_point.unsqueeze(0))
            else: pred = self.net.forward_inference(image, depth, state, target_point.unsqueeze(0))
        pred['acker_speed_branches'],pred['acker_steer_branches'] = pred['acker_speed_branches'].data.cpu().numpy()[0][0],pred['acker_steer_branches'].data.cpu().numpy()[0][0]
        acker_speed_ctrl, acker_steer_ctrl = pred['acker_speed_branches']*self.config.max_speed, pred['acker_steer_branches']*self.config.max_steer
        pred_wp = pred['pred_wp'][0].data.cpu().numpy()
        #The Y axis in Carla/Unreal Engine has the inverse convention, transform for clearer estimation of angle and visualization
        # pred_wp = np.array([[point[0], -point[1]] for point in pred_wp])
        steer, speed = acker_steer_ctrl, acker_speed_ctrl
        if self.inverted_axis:
            steer *= -1
            pred_wp[:, 1] *= -1 
        #The robot is controlled by the ctrl branch, uncomment netx part to use also the waypoints.
        # target_point = target_point.squeeze().data.cpu().numpy()
        # target_point[1] *= -1
        # acker_speed_traj, acker_steer_traj = self.net.control_pid(pred_wp, target_point)

        # if self.status == 0: self.alpha = 0.3
        # else: self.alpha = 0.7
        
        # steer = self.alpha*acker_steer_ctrl + (1-self.alpha)*acker_steer_traj
        # speed = self.alpha*acker_speed_ctrl + (1-self.alpha)*acker_speed_traj


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

        drive_message=AckermannDrive()
        drive_message.speed = float(speed)
        drive_message.steering_angle = float(steer)
        
        d=math.sqrt((self.tick_data['target_point'][0]-self.previous_pose[0])**2+(self.tick_data['target_point'][1]-self.previous_pose[1])**2)
        #Try to avoid local minimum/command
        if d > MINIMUM_DISTANCE:
            self.time_local_minimum=self.tick_data['clock']
            self.previous_pose=self.tick_data['target_point']
        
        if self.tick_data['clock']-self.time_local_minimum>MAX_TIME_STOPPED:
            if abs(self.tick_data["steer"]-self.steer_local_minimum)<0.05:
                self.time_local_minimum=self.tick_data['clock']
                self.steer_local_minimum*=-1
            else:
                drive_message.steering_angle=self.config.max_steer*self.steer_local_minimum
        #Steering angle in radians
        drive_message.steering_angle=drive_message.steering_angle*math.pi/180

        #Check safety stop
        closest_distance= self.check_near_collision(self.tick_data["depth"],drive_message.speed<0)
        distance_id = 0 if drive_message.speed>0 else 1 
        if closest_distance<NEAR_COLLISION_DISTANCE2[distance_id]:
            drive_message.speed=0.0
            print("Emergency stop",closest_distance, flush=True)
        elif closest_distance<NEAR_COLLISION_DISTANCE[distance_id]:
            direction = 1 if drive_message.speed>0 else -1
            drive_message.speed  =  min(float(drive_message.speed ), 
                        direction*self.config.max_speed * (closest_distance-NEAR_COLLISION_DISTANCE2[distance_id])/
                        (NEAR_COLLISION_DISTANCE[distance_id]-NEAR_COLLISION_DISTANCE2[distance_id]))
            print("Decelerating",closest_distance, flush=True)
        #Send the waypoints as markers for visualization.
        marker_message=MarkerArray()
        # for i,point in enumerate(pred_wp):
        #     marker=Marker()
        #     marker.header.stamp = self.get_clock().now()
        #     marker.header.frame_id = "map"
        #     marker.type = Marker.SPHERE
        #     marker.action = Marker.ADD
        #     marker.id=i
        #     point=np.array([point[0], point[1], 0])
            
        #     point = np.dot(self.tick_data["rotation"], point) + self.tick_data["position"]
        #     marker.pose.position.x = point[0]
        #     marker.pose.position.y = point[1]
        #     marker.pose.position.z = 0
        #     marker.scale.x = 0.25
        #     marker.scale.y = 0.25
        #     marker.scale.z = 0.25
        #     marker.color.a = 1.0
        #     marker.color.r = 0.9
        #     marker.color.g = 0.8
        #     marker.color.b = 0.0
        #     marker_message.markers.append(marker)

        #Visualize the ackermann cmd in RViZ.
        arrow_marker = Marker()
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.header.frame_id = "map"
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.id = len(marker_message.markers) + 1

        arrow_marker.pose.position.x = self.tick_data["position"][0]
        arrow_marker.pose.position.y = self.tick_data["position"][1]
        arrow_marker.pose.position.z = 0.0
        yaw = math.atan2(self.tick_data["rotation"][1, 0], self.tick_data["rotation"][0, 0]) + drive_message.steering_angle
        r = R.from_euler('xyz', [0, 0, yaw])
        quaternion = r.as_quat()
        arrow_marker.pose.orientation.x = quaternion[0]
        arrow_marker.pose.orientation.y = quaternion[1]
        arrow_marker.pose.orientation.z = quaternion[2]
        arrow_marker.pose.orientation.w = quaternion[3]

        arrow_marker.scale.x = drive_message.speed
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.1

        arrow_marker.color.a = 1.0
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.0

        marker_message.markers.append(arrow_marker)
        self.drive_message = drive_message
        self.marker_message = marker_message

        self.tick_data = None #Wait for next message

    def goal_callback(self, goal:PoseWithCovarianceStamped):
        self.goal = np.array([goal.pose.pose.position.x, goal.pose.pose.position.y,0])

    # The state message doesn't have the timestamp.
    def state_callback(self, state:AckermannDrive):
        self.state = state

    def publish_step(self):  
        message_deg = AckermannDrive()
        message_deg.speed=self.drive_message.speed
        message_deg.steering_angle = self.drive_message.steering_angle*180/math.pi
        self.drive_deg_pub.publish(message_deg)
        self.drive_rad_pub.publish(self.drive_message)
        self.marker_pub.publish(self.marker_message)
        

    def check_near_collision(self, depth, reverse=False):
        min_distance = 50.0  # Max range is 50 meters
        depth = depth * 50.0 / 65535.0  # Convert to meters
        depth = 50.0 - depth  # The close objects are white, the far objects are black, so it is inverted.

        # Apply Gaussian blur to smooth out the depth image
        depth = cv2.GaussianBlur(depth, (5, 5), 0)

        if reverse:
            for col in itertools.chain(range(0, 125), range(1923, 2048,)):
                for row in range(42, 128):
                    if depth[row,col] < min_distance:
                        min_distance = depth[row,col]
        else:
            for col in range(875, 1125):
                for row in range(42, 128):
                    if depth[row,col] < min_distance:
                        min_distance = depth[row,col]
        return min_distance
    
    def prepare_image(self, tick_data):
        image,depth=None, None
        
        if self.model != "depth":
            image=tick_data['rgb'].astype(np.uint8)
            width = image.shape[1]
            height = image.shape[0]
            crop_w, crop_h = self.config.input_rgb_resolution
            start_y = height // 2 - crop_h // 2
            start_x = width // 2 - crop_w // 2
            cropped_image = image[start_y:start_y + crop_h, start_x:start_x + crop_w]
            image=self._im_transform(cropped_image)
            image=image.to('cuda', dtype=torch.float32)
            image=image.unsqueeze(0)

        if self.model != "rgb":
            depth=tick_data['depth']
            height = depth.shape[0]
            width = depth.shape[1]
            crop_w, crop_h = self.config.input_range_resolution
            start_y = height // 2 - crop_h // 2
            start_x = width // 2 - crop_w // 2
            cropped_depth=depth[start_y:start_y + crop_h, start_x:start_x + crop_w]
            cropped_depth=self._depth_transform(cropped_depth.astype(np.float32))
            depth = cropped_depth.to('cuda', dtype=torch.float32).unsqueeze(0)

        return image, depth
    

    def destroy(self):
        del self.net

    
def main(args=None):
    rclpy.init(args=args)
    actor=Agent()
    print("Network model loaded", flush=True)
    
    try:
        while True:
            rclpy.spin_once(actor)
            actor.publish_step()
    except KeyboardInterrupt:
        pass
    actor.destroy_node()
    rclpy.shutdown()

