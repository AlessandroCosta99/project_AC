#! /usr/bin/env python3
#Author: ALESSANDRO COSTA 02/24
import sys
import math
import random
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy

#ROS
import message_filters
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

#Optimizer
from scipy.optimize import Bounds
from scipy.spatial import distance
from scipy.optimize import minimize
from scipy.optimize import BFGS
from scipy.optimize import LinearConstraint
from scipy.optimize import NonlinearConstraint

## Pytorch
import torch
import torch.onnx
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point
from franka_msgs.msg import FrankaState

#Server
from mpc_control.srv import predictor, predictorResponse


class RobotMPC():
    def __init__(self):
        rospy.init_node("mpc_node", anonymous=True)
        self.prediction_horizon = 10
        self.center_finger = 0.0
        self.d_x = 0.125
        self.d_y = 0.035
        self.x_f = 0.65
        self.y_f = -0.23
        self.d = 0.0005
        self.target_position = np.array([self.x_f, self.y_f])

    def init_sub(self):
        self.initial_position_sub = rospy.Subscriber("franka_state_controller/franka_states", FrankaState, self.franka_state_callback)
        self.robot_ee_pose = message_filters.Subscriber("franka_state_controller/franka_states", FrankaState)
        self.stem_pose_sub = message_filters.Subscriber('/stem_pose',Float64MultiArray)
        sync_sub = [self.robot_ee_pose, self.stem_pose_sub]
        sync_cb = message_filters.ApproximateTimeSynchronizer(sync_sub,  10, 0.1, allow_headerless=True) 
        sync_cb.registerCallback(self.robot_callback)

    def franka_state_callback(self, msg):
        self.robot_pose_init = Float64MultiArray()
        self.robot_pose_init.data = [0.0] *  2
        self.robot_pose_init.data[0] = msg.O_T_EE[12]   # x
        self.robot_pose_init.data[1] = msg.O_T_EE[13]   # y
        #update initial position with the data received
        self.initial_position = np.array([self.robot_pose_init.data[0], self.robot_pose_init.data[1]])
        self.initial_position_sub.unregister()
        #self.initial_position_sub = None


    def robot_callback(self, robot_ee_pose_msg):
        self.finger_pose = np.array(self.ee_to_finger_frame(robot_ee_pose_msg.O_T_EE[12], robot_ee_pose_msg.O_T_EE[13])) 

    def ee_to_finger_frame(self, x, y):
        x_f = x + self.d_x
        y_f = y + self.d_y
        return x_f, y_f
    
    def finger_to_ee_frame(self, x, y):
        x_ee = x - self.d_x
        y_ee = y - self.d_y
        return x_ee, y_ee

    def gen_opt_traj(self):
        initial_theta = np.zeros(self.prediction_horizon-1)
        bounds = None
        result = minimize(self.obj_func, initial_theta, bounds=bounds,method='BFGS')
        optimal_theta = result.x
        return optimal_theta
    
    def circular_to_cartesian(self,theta_values):
        x = np.zeros_like(theta_values+1)
        y = np.zeros_like(theta_values+1)
        for i in range(len(theta_values)):
            if i == 0:
                x[i] = self.initial_position[0]
                y[i] = self.initial_position[1]

            else:
                x[i] = x[i-1] + self.d * np.cos(theta_values[i])
                y[i] = y[i-1] + self.d * np.sin(theta_values[i])
        print("x= {}, y = {}".format(x,y))
        return np.column_stack((x, y))  

    def obj_func(self, theta):
        rospy.wait_for_service("predictor")
        try: 
            candidate_actions = self.circular_to_cartesian(theta)
            predict = rospy.ServiceProxy("predictor", predictor)
            predicted_stem_position = predict(candidate_actions)
            rospy.loginfo(predicted_stem_position.stem_pose)
            pred_berry_pose = np.array(self.finger_pose[0]+predicted_stem_position, self.finger_pose[1])


        except rospy.ServiceException as e:
            print("service exeption %s", e)

        return  np.linalg.norm(self.target_position-pred_berry_pose)**2 + (self.center_finger - predicted_stem_position)**2


if __name__ == '__main__':
    mpc = RobotMPC()