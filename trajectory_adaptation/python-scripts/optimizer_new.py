#! /usr/bin/env python3
#Author: ALESSANDRO COSTA 02/24
import sys
import math
import random
import time
import pandas as pd
import numpy as np
import scipy

#ROS
import moveit_commander
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

#Plots
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class RobotController():
    def __init__(self):
        self.time_step = 0
        self.prev_time_step = 0
        self.x_f = 0.6
        self.y_f = -0.23
        self.num_int_points = 10
        self.trajectory_history = []
        self.cost_history = []
        self.stop = False

        self.goal_pose = PoseStamped()
        self.target_position = np.array([self.x_f, self.y_f])
        self.points = []
        self.ee_coordinates = []
        self.optimal_trajectory_history = []
        self.d = 0.005
        self.initial_position =  np.array([0.0,  0.0])#, 0.75])
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=100)
        self.candidate_actions_pub = rospy.Publisher('/candidate_action', Float64MultiArray, queue_size=10)
        self.dist_from_center = 0.0
        self.save_path = '/home/alessandro/tactile_control_ale_ws/src/data_collection/results_data/001'   #<-- update the last folder for every new test

        self.init_sub()
        if (self.initial_position[0] != 2000):
            self.loop()

    def init_sub(self):
        self.stem_pose_sub = rospy.Subscriber('/stem_pose',Float64MultiArray, self.stem_pose_callback)
        self.initial_position_sub = rospy.Subscriber("franka_state_controller/franka_states", FrankaState, self.franka_state_callback)
        self.strawberry_pose_sub = rospy.Subscriber('/predicted_strawberry_positions', Float64MultiArray, self.predicted_strawberry_pose_cb)

    def franka_state_callback(self, msg):
        self.initial_position[0] = msg.O_T_EE[12]
        self.initial_position[1] = msg.O_T_EE[13]
        #self.initial_position[2] = msg.O_T_EE[14]
        #print(self.initial_position)
        #update initial position with the data received
        self.initial_position_sub.unregister()
        self.initial_position_sub = None

    def predicted_strawberry_pose_cb(self, pred):
        predicted_position = np.array(pred.data).reshape((10, 2))


    def stem_pose_callback(self, stem_pose):
        center_hf = 0.20  #0.0 
        self.dist_from_center = center_hf - stem_pose.data[0]
        print(self.dist_from_center)


    def cost_callback(self, theta_values):    #usefull for tracking the cost value during the optimization
        points = self.circular_to_cartesian(theta_values)
        cost = self.calculate_cost(points)
        self.trajectory_history.append(points.copy())
        self.cost_history.append(cost)
    
    def gen_opt_traj(self):
        initial_theta = np.zeros(self.num_int_points + 1)
        bounds = None
        result = minimize(self.obj_func, initial_theta, bounds=bounds, callback=self.cost_callback, method='BFGS')#,xrtol = "0.01")
        optimal_theta = result.x
        #print(optimal_theta)
        return optimal_theta

    def obj_func(self, theta_values):
        points = self.circular_to_cartesian(theta_values)
        cost = self.calculate_cost(points)
        return cost
       
    def calculate_cost(self, points):
        norms = []
        for p1, p2 in zip(points, self.target_position):
            norm = np.linalg.norm(p1 - p2)**2
            norms.append(norm)
        return self.dist_from_center**2  


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
    
    def sum_distances_from_matrix(self, matrix):
        sum_of_distances =  0
        for row in matrix:
            distance = self.distance_from_point_to_line(row[0], row[1])**2
            sum_of_distances += distance
        return sum_of_distances
    
    def circular_to_cartesian(self,theta_values):
        x = np.zeros(self.num_int_points+2)
        y = np.zeros(self.num_int_points+2)
        #z = np.zeros(self.num_int_points+2)
        for i in range(self.num_int_points+2):
            if i == 0:
                x[i] = self.initial_position[0]
                y[i] = self.initial_position[1]

            else:
                x[i] = x[i-1] + self.d * np.cos(theta_values[i-1])
                y[i] = y[i-1] + self.d * np.sin(theta_values[i-1])

        return np.column_stack((x, y))
    
    def stack_constant_actions(self):
        optimal_actions = self.optimal_trajectory[1:11,:]
        constant_values = np.array([[0.7], [0.726858], [0.0265717], [0.686072]])
        new_columns = np.column_stack([constant_values[i] * np.ones((optimal_actions.shape[0],  1)) for i in range(4)])
        candidate_actions_full = np.hstack((optimal_actions, new_columns))
        self.pub_candidate_actions(candidate_actions_full)

    def pub_candidate_actions(self, actions):  #this goes to the stem_pose_predictor
        candidate_actions = Float64MultiArray()
        candidate_actions.data = actions.flatten().tolist()
        self.candidate_actions_pub.publish(candidate_actions)
    
    def pub_next_pose(self):  #this goes to the robot
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "panda_link0"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x =  self.optimal_trajectory[1,0] # Set x coordinate
        pose_msg.pose.position.y =  self.optimal_trajectory[1,1] # Set y coordinate
        pose_msg.pose.position.z =  0.7
        self.target_pose_pub.publish(pose_msg)

    def loop(self):
        rate=rospy.Rate(11)  
        print("ciao")
        while not rospy.is_shutdown():
            self.opt_theta = self.gen_opt_traj()
            self.optimal_trajectory = self.circular_to_cartesian(self.opt_theta)
            print(self.optimal_trajectory)
            self.pub_next_pose()
            #print(self.optimal_trajectory[0])
            self.stack_constant_actions()

            self.initial_position = self.optimal_trajectory[1]
            #print(self.initial_position)
            # if self.initial_position[0] - self.target_position[0] < 0.005:  #maybe change here
            #     print("Reached target position.")
            #     self.target_pose_pub.unregister()  ##check this, may not work or create problem
            #     break

    def save_data(self):
        np.save(self.save_path + "optimal_trajectory.npy", self.optimal_trajectory)


if __name__ == '__main__':
    rospy.init_node('optimizer') # , anonymous=True, disable_signals=True)
    mpc = RobotController()
    rospy.spin()
