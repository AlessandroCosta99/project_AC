#!/usr/bin/env python3
import tf
import os
import sys
import time
import rospy
import select
import datetime
import actionlib
import numpy as np
import termios, tty
import pandas as pd
import message_filters
from cv_bridge import CvBridge

from std_msgs.msg import Int16MultiArray
from pynput.keyboard import Key, Listener
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Image

class RobotReader(object):
	def __init__(self):
		super(RobotReader, self).__init__()
		rospy.init_node('data_collection_client', anonymous=True, disable_signals=False)
		self.settings = termios.tcgetattr(sys.stdin)
		self.rate_hz = 60  #logging frequency
		rate = rospy.Rate(self.rate_hz)
		self.bridge = CvBridge()

		while input("press enter to start saving data, or type ctrl c then n to not: ") != "n":
			self.stop = False
			self.xela_sensor = []
			self.robot_states = []
			self.pendulum_rot = []
			self.times = []
			self.predicted_pendulum_rot = []
			self.color_image_wrist = []
			self.depth_image_wrist = []
			self.listener = Listener(on_press=self.start_collection)
			self.listener.start()
			print(self.stop)
			print("press esc for stopping the collection")
			self.xela_sub = message_filters.Subscriber('/xela_data', Float64MultiArray)
			self.robot_sub = message_filters.Subscriber('/cartesian_impedance_example_controller/panda_robot_state_topic', Float64MultiArray) # cambridge: "/panda_robot_state_topic"
			self.pendulum_sub = message_filters.Subscriber('/cartesian_impedance_example_controller/panda_pendu_state_topic', Float64MultiArray) # cambridge: "/panda_pendu_state_topic"
			# self.pred_pendulum_sub = message_filters.Subscriber('/pose_prediction', Float64MultiArray) 
			# self.image_color_wrist_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
			# self.image_depth_wrist_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
			subscribers = [self.robot_sub, self.xela_sub, self.pendulum_sub]#, self.pred_pendulum_sub]#, self.image_color_wrist_sub]#, self.image_depth_wrist_sub]
			self.start_time = datetime.datetime.now()
			print(self.start_time)
			self.prev_i = 0
			self.i = 1
			self.index__ = 0
			self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=1, slop=0.1, allow_headerless=True)
			self.ts.registerCallback(self.read_robot_data)

			self.t0 = time.time()
			while not rospy.is_shutdown() and self.stop is False:
				self.i += 1
				rate.sleep()
			self.t1 = time.time()
			self.stop = False
			self.stop_time = datetime.datetime.now()
			print(self.stop_time)

			self.rate = (len(self.xela_sensor)) / (self.t1-self.t0-5) ## why -5?

			print("\n Stopped the data collection \n now saving the stored data")
			self.listener.stop()
			# self.robot_sub.unregister()
			# self.xela_sub.unregister()
			self.save_data()

	def read_robot_data(self, robot_data, xela_data, pendulum_data):#, pred_pendulum_data):#, color_image_wrist_data):#, depth_image_wrist_data):
		
		if self.stop == False and self.i != self.prev_i:
			current_time = time.time()
			if robot_data.data[-1] == 1 and (current_time - self.t0) < 60:
				if (current_time - self.t0) % 1 < 0.03:
					print(current_time - self.t0)
					print("logging in process.")
		
				self.prev_i = self.i
				self.index__ +=1
				self.robot_states.append(robot_data.data)
				self.xela_sensor.append(xela_data.data)
				self.pendulum_rot.append(pendulum_data.data)
				time_of_arrival = rospy.Time.now().to_sec()
				self.times.append(time_of_arrival)
				# self.predicted_pendulum_rot.append(pred_pendulum_data.data)
				# self.color_image_wrist.append(color_image_wrist_data)
				# self.depth_image_wrist.append(depth_image_wrist_data)
		

	def check_keyboard(self, key_timeout):
		if self.getKey(key_timeout):
			print("herer")
			self.stop = True
			self.robot_sub.unregister()
			self.xela_sub.unregister()
			self.pendulum_sub.unregister()
			#self.pred_pendulum_sub.unregister()
			# self.image_color_wrist_sub.unregister()
			# self.image_depth_wrist_sub.unregister()

	def start_collection(self, key):
		print("herer")
		if key == Key.esc:
			self.stop = True
			self.listener.stop()
			self.robot_sub.unregister()
			self.xela_sub.unregister()
			self.pendulum_sub.unregister()
			# self.pred_pendulum_sub.unregister()
			# self.image_color_wrist_sub.unregister()
			# self.image_depth_wrist_sub.unregister()

	def format_data_for_saving(self):
		print("Formatting the data")
		self.robot_states = np.array(self.robot_states)
		self.xela_sensor = np.array(self.xela_sensor)
		#self.predicted_pendulum_rot = np.array(self.predicted_pendulum_rot)
		self.times = np.array(self.times)
	
		self.robot_states_formatted = np.reshape(self.robot_states, (len(self.robot_states), len(self.robot_states[0])))
		self.xela_sensor_formatted = np.reshape(self.xela_sensor, (len(self.xela_sensor), len(self.xela_sensor[0])))
		self.pendulum_rot_formatted = np.reshape(self.pendulum_rot, (len(self.pendulum_rot), len(self.pendulum_rot[0])))
		#self.pred_pendulum_rot_formatted = np.reshape(self.predicted_pendulum_rot, (len(self.predicted_pendulum_rot), len(self.predicted_pendulum_rot[0])))
	
	def save_data(self):

		self.format_data_for_saving()

		self.xela_sensor_formatted = np.array(self.xela_sensor_formatted)
		self.xela_sensor_formatted = np.reshape(self.xela_sensor_formatted, (self.xela_sensor_formatted.shape[0], 48))

		print("xela_sensor_formatted length: ", self.xela_sensor_formatted.shape)
		print("robot_states_formatted; ", np.asarray(self.robot_states_formatted).shape)
		print("pendulum_rot_formatted; ", np.asarray(self.pendulum_rot_formatted).shape)
		#print("pred_pendulum_rot_formatted; ", np.asarray(self.pred_pendulum_rot_formatted).shape)
		print("rate: ", self.rate)
		print("shape of times vector: ", self.times.shape)

		T1 = pd.DataFrame(self.xela_sensor_formatted)
		T2 = pd.DataFrame(self.robot_states_formatted)
		T3 = pd.DataFrame(self.pendulum_rot_formatted)
		#T4 = pd.DataFrame(self.pred_pendulum_rot_formatted)
		T5 = pd.DataFrame(self.times)

		# create new folder for this experiment:
		# ------------------------------------------------modify----------------------------------------------------#
		#folder = str('/home/kiyanoush/catkin_ws/src/inverted_pendulum/data/dataset_lincoln/passive_system/' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
		folder = str('/home/jeff_franka_emika/Documents/dataset_14Dec/' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
		# ------------------------------------------------modify-----------------------------------------------------#
		mydir = os.mkdir(folder)

		xela_Sensor_col = ['txl1_x', 'txl1_y', 'txl1_z', 'txl2_x', 'txl2_y', 'txl2_z','txl3_x', 'txl3_y', 'txl3_z','txl4_x', 'txl4_y', 'txl4_z','txl5_x', 'txl5_y', 'txl5_z','txl6_x', 'txl6_y', 'txl6_z',
		'txl7_x', 'txl7_y', 'txl7_z','txl8_x', 'txl8_y', 'txl8_z','txl9_x', 'txl9_y', 'txl9_z','txl10_x', 'txl10_y', 'txl10_z','txl11_x', 'txl11_y', 'txl11_z','txl12_x', 'txl12_y', 'txl12_z',
		'txl13_x', 'txl13_y', 'txl13_z','txl14_x', 'txl14_y', 'txl14_z','txl15_x', 'txl15_y', 'txl15_z','txl16_x', 'txl16_y', 'txl16_z']
		

		robot_states_col = ["q1", "q2", "q3", "q4", "q5", "q6", "q7", "q_d1", "q_d2", "q_d3", "q_d4", "q_d5", "q_d6", "q_d7", \
			  				"dq1", "dq2", "dq3", "dq4", "dq5", "dq6", "dq7", "dq_d1", "dq_d2", "dq_d3", "dq_d4", "dq_d5", \
							"dq_d6", "dq_d7", "ddq_d1", "ddq_d2", "ddq_d3", "ddq_d4", "ddq_d5", "ddq_d6", "ddq_dv",\
							"tau_J1", "tau_J2", "tau_J3", "tau_J4", "tau_J5", "tau_J6", "tau_J7", "tau_J_d1", \
							"tau_J_d2", "tau_J_d3", "tau_J_d4", "tau_J_d5", "tau_J_d6", "tau_J_d7",\
							 "O_T_EE1", "O_T_EE2", "O_T_EE3", "O_T_EE4", "O_T_EE5", "O_T_EE6", "O_T_EE7", "O_T_EE8",\
							 "O_T_EE9", "O_T_EE10", "O_T_EE11", "O_T_EE12", "O_T_EE13", "O_T_EE14", "O_T_EE15", "O_T_EE16", \
							 "O_T_EE_d1", "O_T_EE_d2", "O_T_EE_d3", "O_T_EE_d4", "O_T_EE_d5", "O_T_EE_d6", "O_T_EE_d7", "O_T_EE_d8", \
							 "O_T_EE_d9", "O_T_EE_d10", "O_T_EE_d11", "O_T_EE_d12", "O_T_EE_d13", "O_T_EE_d14", "O_T_EE_d15", "O_T_EE_d16", \
							 "O_T_EE_c1", "O_T_EE_c2", "O_T_EE_c3", "O_T_EE_c4", "O_T_EE_c5", "O_T_EE_c6", "O_T_EE_c7", "O_T_EE_c8", \
							 "O_T_EE_c9", "O_T_EE_c10", "O_T_EE_c11", "O_T_EE_c12", "O_T_EE_c13", "O_T_EE_c14", "O_T_EE_c15", "O_T_EE_c16", \
							 "EE_T_K1", "EE_T_K2", "EE_T_K3", "EE_T_K4", "EE_T_K5", "EE_T_K6", "EE_T_K7", "EE_T_K8", \
							 "EE_T_K9", "EE_T_K10", "EE_T_K11", "EE_T_K12", "EE_T_K13", "EE_T_K14", "EE_T_K15", "EE_T_K16", \
							 "O_F_ext_hat_K1", "O_F_ext_hat_K2", "O_F_ext_hat_K3", "O_F_ext_hat_K4", "O_F_ext_hat_K5", "O_F_ext_hat_K6",\
							 "K_F_ext_hat_K1", "K_F_ext_hat_K2", "K_F_ext_hat_K3", "K_F_ext_hat_K4", "K_F_ext_hat_K5", "K_F_ext_hat_K6",\
							 "m_ee", "m_load", "m_total", "error_1", "error_2", "error_3", "error_4", "error_5", "error_6",\
							 "d_error_1", "d_error_2", "d_error_3", "d_error_4", "d_error_5", "d_error_6",\
							 "impedance_force_x", "impedance_force_y", "impedance_force_z", "impedance_force_alpha", "impedance_force_beta", "impedance_force_gamma",\
							 "jacobian1", "jacobian2", "jacobian3", "jacobian4", "jacobian5", "jacobian6",\
							 "jacobian7", "jacobian8", "jacobian9", "jacobian10", "jacobian11", "jacobian12",\
							 "jacobian13", "jacobian14", "jacobian15", "jacobian16", "jacobian17", "jacobian18",\
							 "jacobian19", "jacobian20", "jacobian21", "jacobian22", "jacobian23", "jacobian24",\
							 "jacobian25", "jacobian26", "jacobian27", "jacobian28", "jacobian29", "jacobian30",\
							 "jacobian31", "jacobian32", "jacobian33", "jacobian34", "jacobian35", "jacobian36",\
							 "jacobian37", "jacobian38", "jacobian39", "jacobian40", "jacobian41", "jacobian42",\
							 "coriolis1", "coriolis2", "coriolis3", "coriolis4", "coriolis5", "coriolis6", "coriolis7",\
							 "period_time", "start_data_collection"]
		


		# pendulum_rot_col = ["alpha_true", "beta_true", "alpha_saturated", "beta_saturated", "d_alpha", "d_beta"]
		pendulum_rot_col = ["alpha_true", "beta_true", "alpha_saturated", "beta_saturated", "d_alpha", "d_beta",\
					  		"predicted_alpha", "predicted_beta", "pred_alpha_dot", "pred_beta_dot"]
		pred_pendulum_rot_col = ["alpha_pred", "beta_pred"]

		times_col = ["time of collection"]

		T1.to_csv(folder + '/xela_sensor.csv', header=xela_Sensor_col, index=False)
		T2.to_csv(folder + '/robot_state.csv', header=robot_states_col, index=False)
		T3.to_csv(folder + '/pendulum_state.csv', header=pendulum_rot_col, index=False)
		#T4.to_csv(folder + '/pendulum_prediction.csv', header=pred_pendulum_rot_col, index=False)
		T5.to_csv(folder + '/times.csv', header = times_col, index = False)

		# np.save(folder + '/color_image_wrist.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.color_image_wrist]))
		# np.save(folder + '/depth_image_wrist.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.depth_image_wrist]))

if __name__ == "__main__":
	robot_reader = RobotReader()