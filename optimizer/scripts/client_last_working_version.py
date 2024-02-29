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
import rospy
from std_msgs.msg import Float64MultiArray
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

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

#Server
from optimizer.srv import pred, predResponse


class RobotMPC():
    def __init__(self):
        rospy.init_node("mpc_node", anonymous=True)
        self.prediction_horizon = 2
        self.center_finger = 0.0
        self.d_x = 0.125
        self.d_y = 0.035
        self.x_f = 0.60
        self.y_f = -0.23
        self.d = 0.05
        self.target_position = np.array([self.x_f, self.y_f])
        self.initial_position = np.array([0.0,0.0])
        self.optimal_traj_pub = rospy.Publisher('/next_pose', PoseStamped, queue_size=1)   #for the robot ( i need this type of data)
        self.init_sub()
        self.i = 0
        self.loop()

    def init_sub(self):
        self.initial_position_sub = rospy.Subscriber("/cartesian_impedance_example_controller/panda_robot_state_topic", Float64MultiArray, self.franka_state_callback)
        self.robot_ee_pose = rospy.Subscriber("/cartesian_impedance_example_controller/panda_robot_state_topic", Float64MultiArray, self.robot_callback)

    def franka_state_callback(self, msg):
        self.robot_pose_init = Float64MultiArray()
        self.robot_pose_init.data = [0.0] * 2
        self.robot_pose_init.data[0] = msg.data[12]   # x
        self.robot_pose_init.data[1] = msg.data[13]   # y
        #update initial position with the data received
        self.initial_position = np.array([self.robot_pose_init.data[0], self.robot_pose_init.data[1]])
        self.initial_position_sub.unregister()
        #self.initial_position_sub = None


    def robot_callback(self, robot_ee_pose_msg):
        self.finger_pose = np.array(self.ee_to_finger_frame(robot_ee_pose_msg.data[12], robot_ee_pose_msg.data[13])) 
        # print("x_{} finger: {}".format(self.i, self.finger_pose[0]))
        # print("y_{} finger: {}".format(self.i, self.finger_pose[1]))

    def ee_to_finger_frame(self, x, y):
        x_f = x + self.d_x
        y_f = y + self.d_y
        return x_f, y_f
    
    def finger_to_ee_frame(self, x, y):
        x_ee = x - self.d_x
        y_ee = y - self.d_y
        return x_ee, y_ee

    def gen_opt_traj(self):
        initial_theta = np.zeros(self.prediction_horizon)
        bounds = [(-math.pi/6, math.pi/6),(-math.pi/6, math.pi/6)]
        optimizer_options = {'verbose': 3, 'xtol':1e-08, 'gtol':1e-08,  'maxiter':3, 'disp':True}
        result = minimize(self.obj_func, initial_theta, bounds=bounds, method='trust-constr'  ,options= optimizer_options)
        optimal_theta = result.x
        #print(" if i exite the minimization loop this is the optimal_theta:", optimal_theta)
        return optimal_theta
    
    def circular_to_cartesian(self,theta_values):
        x = np.zeros_like(theta_values+1)
        y = np.zeros_like(theta_values+1)
        following_null_actions = np.zeros(54)
        cand =[]
        constant_values = np.array([0.8, 114.38760473,  84.13176385,  65.16388286]) #last three are euler angles
        for i in range(len(theta_values)):
            if i == 0:
                x[i] = self.initial_position[0]
                y[i] = self.initial_position[1]

            else:
                print(theta_values[i-1])
                x[i] = x[i-1] + self.d * np.cos(theta_values[i-1])
                y[i] = y[i-1] + self.d * np.sin(theta_values[i-1])
                cand = np.append(cand, x[i])
                cand = np.append(cand, y[i])
                cand = np.append(cand, constant_values)
        cand =np.append(cand,following_null_actions)
        return cand 

    def obj_func(self, theta):
        rospy.wait_for_service("predictor")

        try:
            candidate_actions = Float64MultiArray(data = self.circular_to_cartesian(theta))
            predict = rospy.ServiceProxy("predictor", pred)
            #print(candidate_actions)
            predicted_stem_position = predict(candidate_actions)
            #print("Response of the service: ",predicted_stem_position.stem_pose.data)
            #rospy.loginfo(predicted_stem_position.stem_pose)  # it gives me a float multy array msg layout
            pred_berry_pose = np.array(self.finger_pose[0]+predicted_stem_position.stem_pose.data, self.finger_pose[1])
        except rospy.ServiceException as e:
            print("service exeption %s", e)

        cost = np.linalg.norm(self.target_position-pred_berry_pose)**2 + (self.center_finger - predicted_stem_position.stem_pose.data[0])**2
        return cost

    
    def pub_next_pose(self):  #this goes to the robot
        print("pub_pose")
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "panda_link0"
        pose_msg.header.stamp = rospy.Time.now()
        #print("commanded_x = {}, commanded_y = {}".format(self.optimal_trajectory[1,0],self.optimal_trajectory[1,1]))
        pose_msg.pose.position.x =  self.optimal_trajectory[0] #self.optimal_trajectory[1,0] # Set x coordinate
        pose_msg.pose.position.y =  self.optimal_trajectory[1]  # Set y coordinate
        pose_msg.pose.position.z =  0.8 #self.robot_pose_init.data[2]
        self.optimal_traj_pub.publish(pose_msg)

    def loop(self):
        print("Let's start")
        rate=rospy.Rate(1000)    

        while not rospy.is_shutdown():

            try:
                self.opt_theta = self.gen_opt_traj()
                print("Minimization done -> optimal_theta = ", self.opt_theta)
                self.optimal_trajectory = self.circular_to_cartesian(self.opt_theta)
                print(self.optimal_trajectory)
                print("opt_x = {}, opt_y = {}".format(self.optimal_trajectory[0],self.optimal_trajectory[1]))
                self.pub_next_pose()
                #self.initial_position = self.optimal_trajectory[1]
                print("-------------------------------------------------")
                rate.sleep()

            except KeyboardInterrupt:
                break

if __name__ == '__main__':
    mpc = RobotMPC()