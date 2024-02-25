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

class RobotController():
    def __init__(self):
        rospy.init_node('optimizer')
        self.time_step = 0
        self.prev_time_step = 0
        self.pred_berry_pose = [367.0,200.0]
        #Varuables initialization
        self.dist_from_center = 0.0
        self.x_f = 828 #image frame
        self.y_f = 241 #image frame
        self.pred_horizon = 1    #<----------- change here
        self.trajectory_history = []
        self.stem_error_history = []
        self.predicted_strawberry_position_history = []
        self.target_position = np.array([self.x_f, self.y_f])
        self.points = []
        self.optimal_trajectory_history = []
        self.d = 0.005
        self.initial_robot_position =  np.array([0.0,  0.0, 0.0])

        #Publishers
        self.optimal_traj_pub = rospy.Publisher('/next_pose', PoseStamped, queue_size=100)   #for the robot ( i need this type of data)
        self.candidate_actions_pub = rospy.Publisher('/candidate_action', Float64MultiArray, queue_size=10)   #for the predictors
        self.dist_from_center = 0.0
        self.save_path = '/home/alessandro/tactile_control_ale_ws/src/data_collection/results_data/001' #<-- update the last folder for every new test

        self.init_sub()
        self.loop()

    def init_sub(self):
        self.initial_position_sub = rospy.Subscriber("franka_state_controller/franka_states", FrankaState, self.franka_state_callback)
        #self.stem_pose_sub = message_filters.Subscriber('/stem_pose',Float64MultiArray)
        self.strawberry_pose_sub = message_filters.Subscriber('/predicted_strawberry_positions', Float64MultiArray)
        sync_sub = [ self.strawberry_pose_sub] #self.stem_pose_sub,
        sync_cb = message_filters.ApproximateTimeSynchronizer(sync_sub,  10, 0.1, allow_headerless=True) 
        sync_cb.registerCallback(self.callback)

    def franka_state_callback(self, msg):
        self.robot_pose_init = Float64MultiArray()
        self.robot_pose_init.data = [0.0] * 3
        self.robot_pose_init.data[0] = msg.O_T_EE[12]
        #print(self.robot_pose_init.data[0])
        self.robot_pose_init.data[1] = msg.O_T_EE[13]
        self.robot_pose_init.data[2] = msg.O_T_EE[14]
        #print(self.robot_pose_init.data[2])
        #update initial position with the data received
        self.initial_position = np.array([self.robot_pose_init.data[0], self.robot_pose_init.data[1], self.robot_pose_init.data[2]])
        self.initial_position_sub.unregister()
        self.initial_position_sub = None

    def callback(self,  berry_pred_msg):  #stem_pose_msg,
        center_hf = 0.00 #maybe 0.2
        #self.dist_from_center = center_hf - stem_pose_msg.data[0]
        #self.stem_error_history.append(self.dist_from_center)
        self.pred_berry_pose = np.array(berry_pred_msg.data).reshape((1, 2))
 

    def gen_opt_traj(self):
        initial_theta = np.zeros(self.pred_horizon)
        bounds = None
        result = minimize(self.obj_func, initial_theta, bounds=bounds,method='BFGS')
        optimal_theta = result.x
        return optimal_theta

    def obj_func(self, theta):
        return np.linalg.norm(self.target_position-self.pred_berry_pose)**2 #+ self.dist_from_center**2
        #target_position is in image frame ----------------------------- distance between stem localization and center of the finger
        

    def line_equation(self, x1, y1, x2, y2):
        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
        return m, b

    def distance_from_point_to_line(self, x, y):     #  here i will pass the predicted position as x and y
        x1 = self.initial_position[0]
        y1 = self.initial_position[1]
        x2 = self.x_f
        y2 = self.y_f
        m, b = self.line_equation(x1, y1, x2, y2)
        return abs(m*x - y - b) / math.sqrt(m**2 +  1)
    
    def sum_squared_distances(self, matrix):
        sum_of_distances =  0
        for row in matrix:
            distance = self.distance_from_point_to_line(row[0], row[1])**2
            sum_of_distances += distance
        return sum_of_distances
    
    def circular_to_cartesian(self,theta_values):
        x = np.zeros(self.pred_horizon+1)
        y = np.zeros(self.pred_horizon+1)
        z = np.zeros(self.pred_horizon+1)
        for i in range(self.pred_horizon+1):
            if i == 0:
                x[i] = self.initial_robot_position[0]
                y[i] = self.initial_robot_position[1]

            else:
                x[i] = x[i-1] + self.d * np.cos(theta_values[i-1])
                y[i] = y[i-1] + self.d * np.sin(theta_values[i-1])

        return np.column_stack((x, y))    #np.array of shape (n+1,2)
    
    # def stack_constant_actions(self):
    #     optimal_actions = self.optimal_trajectory[1:11,:]
    #     constant_values = np.array([[0.7], [0.726858], [0.0265717,], [0.686072]]) #check this
    #     new_columns = np.column_stack([constant_values[i] * np.ones((optimal_actions.shape[0],  1)) for i in range(4)])
    #     candidate_actions_full = np.hstack((optimal_actions, new_columns))
    #     self.pub_candidate_actions(candidate_actions_full)

    def pub_candidate_actions(self):  # this goes to the stem_pose_predictor
        self.optimal_trajectory = self.circular_to_cartesian(self.opt_theta)
        candidate_actions = Float64MultiArray()
        velocity = (self.optimal_trajectory[1,:]-self.optimal_trajectory[0,:])/0.034
        candidate_actions.data = velocity  #approximated
        self.candidate_actions_pub.publish(candidate_actions)
    
    def pub_next_pose(self):  #this goes to the robot
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "panda_link0"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x =  self.optimal_trajectory[1,0] # Set x coordinate
        pose_msg.pose.position.y =  self.optimal_trajectory[1,1] # Set y coordinate
        pose_msg.pose.position.z =  0.78 #self.robot_pose_init.data[2]
        self.optimal_traj_pub.publish(pose_msg)

    def loop(self):
        rate=rospy.Rate(50)    

        while not rospy.is_shutdown():

            try:
                self.opt_theta = self.gen_opt_traj()
                self.optimal_trajectory = self.circular_to_cartesian(self.opt_theta)
                self.optimal_trajectory_history.append(self.optimal_trajectory[1])
                self.pub_next_pose()
                self.pub_candidate_actions()
                # self.stack_constant_actions()

                self.initial_position = self.optimal_trajectory[1]
                
                # if abs(self.pred_berry_pose[0] - self.target_position[0]) < 1.0:  #maybe change here
                #     #self.target_pose_pub.unregister()  ##check this, may not work or create problem
                #     print("Reached target position.", self.pred_berry_pose[0] - self.target_position[0])
                #     break
                rate.sleep()

            except KeyboardInterrupt:
                break

    def save_data(self):
        np.save(self.save_path + "optimal_trajectory.npy", self.optimal_trajectory_history)
        np.save(self.save_path + "steam_error.npy", self.stem_error_history)
        np.save(self.save_path + "predicted_position.npy", self.predicted_strawberry_position_history)


if __name__ == '__main__':
    mpc = RobotController()
    #mpc.save_data()