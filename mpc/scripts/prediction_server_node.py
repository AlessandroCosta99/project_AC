#! /usr/bin/env python3
import cv2
import time
import rospy
import torch
import numpy as np
import torch.nn as nn
import message_filters
from pickle import load
import PIL.Image as PILImage
from cv_bridge import CvBridge
import torch.nn.functional as F
from openvino.runtime import Core
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import FrankaState

from scipy.spatial.transform import Rotation as R


from ATCVP.ATCVP import Model

#Server
from mpc_control.srv import predictor, predictorResponse


class localisation_model(nn.Module):
    
    def __init__(self):
        super(localisation_model, self).__init__()
        self.conv1 = nn.Conv2d(3, 8, 3, padding=1)
        self.conv2 = nn.Conv2d(8, 16, 3, padding=1)
        self.maxpool = nn.MaxPool2d(2, 2)
        # self.fc1 = nn.Linear(16*64*64, 512) # this is for 256x256
        self.fc1 = nn.Linear(16*16*16, 512)   # this is for 64x64
        self.fc2 = nn.Linear(512, 32)
        self.fc3 = nn.Linear(32, 1)
        self.relu = nn.ReLU()
        self.flatten = nn.Flatten(start_dim=1)
    
    def forward(self, tactile):
        x = self.relu(self.maxpool(self.conv1(tactile)))
        x = self.relu(self.maxpool(self.conv2(x)))
        x = self.flatten(x)
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.fc3(x)

        return x

class PushingController:
	def __init__(self):
		super(PushingController, self).__init__()
		self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
		rospy.init_node("predictor")
		self.time_step = 0.0
		self.stop = 0
		service = rospy.Service("predictor", predictor, self.callback_theta)
		rospy.spin()

		#vVariable initialization
		self.bridge = CvBridge()
		self.localisation 		 = np.zeros((1000, 1)).astype(np.float32)
		self.robot_pose_data 	 = np.zeros((1000, 10)).astype(np.float32)
		self.robot_data_scaled 	 = np.zeros((1000, 5, 6)).astype(np.float32)
		self.action_data_scaled  = np.zeros((1000, 10, 6)).astype(np.float32)
		self.action_concat		 = np.zeros((1000, 15, 6)).astype(np.float32)
		self.haptic_finger_data  = np.zeros((1000, 3, 64, 64)).astype(np.float32)
		self.haptic_finger_data_raw  = np.zeros((1000, 360, 480, 3)).astype(np.float32)
		self.haptic_finger_scaled  = np.zeros((1000, 64, 64, 3)).astype(np.float32)
		self.tactile_predictions = np.zeros((1000, 10, 3, 64, 64)).astype(np.float32)
		self.final_haptic_input = np.zeros((1000, 5, 3, 64, 64))
		self.final_action_input = np.zeros((1000, 15, 6))
		self.localisation = np.zeros((1000, 1))
	

	def load_model(self):
		n_past = 5
		n_future = 10
		model_dir = "/home/alessandro/tactile_control_ale_ws/src/haptic_finger_control/src/ATCVP/blacked/"
		model_name_save_appendix = "ATCVP_model"

		features = dict([("device", self.device), ("n_past", n_past), ("n_future", n_future), ("model_dir", model_dir),\
										("model_name_save_appendix", model_name_save_appendix), ("criterion", nn.MSELoss())])
		self.pred_model = Model(features)
		self.pred_model.load_state_dict(torch.load("/home/alessandro/tactile_control_ale_ws/src/haptic_finger_control/src/ATCVP/blacked/ATCVP_model", map_location='cpu'))
		self.pred_model = self.pred_model#.float()
		self.pred_model.eval()

		self.local_model = localisation_model()
		self.local_model.load_state_dict(torch.load(\
									"/home/alessandro/tactile_control_ale_ws/src/haptic_finger_control/force_localisation/dataset/localisation_cnn_crop64.pth"))
		self.local_model = self.local_model.float()
		self.local_model.eval()
		print("Model loaded")
		self.model_warmup()

	def model_warmup(self):
		for param in self.pred_model.parameters():
			param.grad = None
		for param in self.local_model.parameters():
			param.grad = None
		action_dummy_input = torch.randn(15, 1, 6).float()
		touch_dummy_input  = torch.randn(5, 1, 3, 64, 64).float()
		# scene_dummy_input  = torch.randn(15, 1, 3, 256, 256).float()	
		for _ in range(5):
			x = self.pred_model(action_dummy_input, touch_dummy_input)
		print("Finished model initialisation and warm up ...")

	def load_scalers(self):
		scaler_path = '/home/alessandro/tactile_control_ale_ws/src/haptic_finger_control/scalars'
		self.robot_min_max_scalar = [load(open(scaler_path + '/robot_min_max_scalar_'+feature +'.pkl', 'rb'))\
															 for feature in ['px', 'py', 'pz', 'ex', 'ey', 'ez']]
		print("Scalers loaded")
	
	def callback(self, robot_poes_msg, haptic_finger_msg):
		# self.stop = robot_poes_msg.data[-1]
		print(self.time_step)
		if self.stop == 0:

			rot_mat = R.from_matrix([[robot_poes_msg.O_T_EE[0], robot_poes_msg.O_T_EE[4], robot_poes_msg.O_T_EE[8]],\
									[robot_poes_msg.O_T_EE[1], robot_poes_msg.O_T_EE[5], robot_poes_msg.O_T_EE[9]],\
									[robot_poes_msg.O_T_EE[2], robot_poes_msg.O_T_EE[6], robot_poes_msg.O_T_EE[10]]])	

			euler = rot_mat.as_euler('zyx', degrees=True)
			quat  = rot_mat.as_quat()

			self.robot_pose_data[self.time_step] = np.array([robot_poes_msg.O_T_EE[12], robot_poes_msg.O_T_EE[13], robot_poes_msg.O_T_EE[14],\
														 euler[0], euler[1], euler[2],\
														quat[0], quat[1], quat[2], quat[3]])

			#self.robot_vel_data[self.time_step] = np.array(robot_vel_msg.data)

			haptic_finger_img = self.bridge.imgmsg_to_cv2(haptic_finger_msg, desired_encoding='passthrough')
			haptic_finger_img = cv2.rectangle(haptic_finger_img,(0,230),(480,480),(0,0,0),-1)
			self.haptic_finger_data_raw[self.time_step] = haptic_finger_img
			haptic_finger_img = PILImage.fromarray(haptic_finger_img).resize((64, 64), PILImage.LANCZOS)
			haptic_finger_img = np.array(haptic_finger_img).astype(np.uint8)
			haptic_finger_img = haptic_finger_img.astype(np.float32)

			haptic_finger_img = haptic_finger_img / 255.0
			self.haptic_finger_scaled[self.time_step] = haptic_finger_img[np.newaxis, :, : , :]

			if self.time_step > 6:

				scaled_robot  = np.zeros_like(self.robot_pose_data[self.time_step-5 : self.time_step, :6]).astype(np.float32)


				for index, min_max_scalar in enumerate(self.robot_min_max_scalar):
					scaled_robot[:, index] = np.squeeze(min_max_scalar.transform(self.robot_pose_data[self.time_step-5 : self.time_step, :6][:, index].reshape(-1, 1)))

					#instead of self.action_data
					scaled_candidate_action[:, index] = np.squeeze(min_max_scalar.transform(candidate_action[:, index].reshape(-1, 1)))

				self.robot_data_scaled[self.time_step] = scaled_robot
				self.action_data_scaled[self.time_step] = scaled_candidate_action
				scaled_action_r = np.concatenate((scaled_robot, scaled_candidate_action), axis=0).astype(np.float32)
				self.action_concat[self.time_step] = scaled_action_r

				self.scaled_haptic = torch.from_numpy(self.haptic_finger_scaled[self.time_step-6:self.time_step-1]).unsqueeze(1).permute((0, 1, 4, 2, 3)) # shape: 5, 1, 3, 64, 64
				self.scaled_action = torch.from_numpy(self.action_concat[self.time_step]).unsqueeze(1)

				self.final_haptic_input[self.time_step] = self.scaled_haptic[:, 0, :, :, :]
				self.final_action_input[self.time_step] = self.scaled_action[:, 0, :]

		self.time_step +=1
	
	def predict_tactile_seq(self):

		tactile_prediction = self.pred_model.forward(self.scaled_action, self.scaled_haptic)

		return tactile_prediction
	
	def pred_to_stem_detection(self, tac_pred_frame):
		# tac_norm = tac_pred_frame - self.haptic_finger_data[0] / 255.0
		tac_norm = tac_pred_frame[2] - tac_pred_frame[6]
		tactile_tensor = tac_norm.unsqueeze(0) # torch
		stem_pose = self.local_model(tactile_tensor.float()).item() # torch
		self.localisation[self.time_step] = stem_pose

		return stem_pose


	def callback_candidate_actions(self, request):

		 
		
		tactile_prediction_seq = self.predict_tactile_seq().squeeze().detach()
		self.tactile_predictions[self.time_step] = tactile_prediction_seq

		stem_pose = self.pred_to_stem_detection(tactile_prediction_seq)
		
		return predictorResponse(stem_pose)
	
		
	


if __name__ == '__main__':
	pc = PushingController()
	rospy.spin()