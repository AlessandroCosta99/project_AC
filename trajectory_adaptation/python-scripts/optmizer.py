#! /usr/bin/env python3
import sys
import math
import random
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy

#ROS
import moveit_commander
import message_filters
import rospy
from std_msgs.msg import Float64MultiArray, Float64, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Point, Pose, PoseStamped
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


class FrankaRobot(object):
    def __init__(self):
        super(FrankaRobot, self).__init__()

    def get_robot_task_state(self):
        robot_ee_pose = self.move_group.get_current_pose().pose
        return [[robot_ee_pose.position.x, robot_ee_pose.position.y, \
                robot_ee_pose.position.z], [robot_ee_pose.orientation.x, \
                robot_ee_pose.orientation.y, robot_ee_pose.orientation.z, \
                robot_ee_pose.orientation.w]]

class RobotController():
    def __init__(self):
        self.time_step = 0
        self.prev_time_step = 0
        self.x_f = 0.55 
        self.y_f = -0.30 
        self.num_int_points = 10
        self.trajectory_history = []
        self.cost_history = []
        self.stop = False

        self.goal_pose = PoseStamped()
        self.target_position = np.array([self.x_f, self.y_f])
        self.points = []
        self.ee_coordinates = []
        self.optimal_trajectory_history = []
        # self.start_position = np.array([self.x0, self.y0])
        # self.d = np.linalg.norm(self.target_position - self.start_position) / (self.num_int_points+1)
        self.d = 0.001
        self.initial_position =  np.array([0.0,  0.0, 0.0])
        self.optimal_traj_pub = rospy.Publisher('/next_pose', PoseStamped, queue_size=100)
        self.init_sub()
        self.center_hf = 0
        self.loop()
        #self.plot_trajectory()

    def init_sub(self):
        # self.stem_pose_sub = rospy.Subscriber('/stem_pose',Float64MultiArray, self.stem_pose_callback)
        self.initial_position_sub = rospy.Subscriber("franka_state_controller/franka_states", FrankaState, self.franka_state_callback)

    def franka_state_callback(self, msg):
        self.robot_pose_init = Float64MultiArray()
        self.robot_pose_init.data = [0.0] *  3
        self.robot_pose_init.data[0] = msg.O_T_EE[12]
        self.robot_pose_init.data[1] = msg.O_T_EE[13]
        self.robot_pose_init.data[2] = msg.O_T_EE[14]
        #update initial position with the data received
        self.initial_position = np.array([self.robot_pose_init.data[0], self.robot_pose_init.data[1], self.robot_pose_init.data[2]])
        self.initial_position_sub.unregister()
        self.initial_position_sub = None

    def stem_pose_callback(self, stem_pose):
        self.dist_from_center = self.center_hf - stem_pose.data

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
        return optimal_theta

    def obj_func(self, theta_values):
        points = self.circular_to_cartesian(theta_values)
        cost = self.calculate_cost(points)
        return cost
       
    def calculate_cost(self, points):
        return np.sum(np.linalg.norm(points - self.target_position, axis=1)**2) #+ self.dist_from_center**2

    def circular_to_cartesian(self,theta_values): 
        x = np.zeros(self.num_int_points+2)
        y = np.zeros(self.num_int_points+2)
        z = np.zeros(self.num_int_points+2)
        for i in range(self.num_int_points+2):
            if i == 0:
                x[i] = self.initial_position[0]
                y[i] = self.initial_position[1]

            else:
                x[i] = x[i-1] + self.d * np.cos(theta_values[i-1])
                y[i] = y[i-1] + self.d * np.sin(theta_values[i-1])

           
        return np.column_stack((x, y))
    
    def pub_next_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "panda_link0"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x =  self.optimal_trajectory[1,0] # Set x coordinate
        pose_msg.pose.position.y =  self.optimal_trajectory[1,1] # Set y coordinate
        pose_msg.pose.position.z =  self.robot_pose_init.data[2]
        self.optimal_traj_pub.publish(pose_msg)

    def loop(self):
        rate=rospy.Rate(1001)    #not sure if it will work

        while not rospy.is_shutdown():
            self.opt_theta = self.gen_opt_traj()
            self.optimal_trajectory = self.circular_to_cartesian(self.opt_theta)
            print(self.initial_position)
            self.pub_next_pose()


            self.initial_position = self.optimal_trajectory[1]
            if self.initial_position[0] - self.target_position[0] < 0.005:
                print("Reached target position.")
                break
    
        print(self.initial_position)
        # if np.all(abs(self.initial_position - self.target_position) < 0.002):

                    				

    def plot_trajectory(self):
        # Plot the trajectory at each step
        #for i, trajectory in enumerate(self.trajectory_history):
        #   plt.plot(trajectory[:, 0], trajectory[:, 1], label=f'Step {i}')

        # Plot the final optimized trajectory
        plt.scatter(self.optimized_trajectory[:, 0], self.optimized_trajectory[:, 1], label='Optimized', color='red', marker='x')

        # Plot the target position
        plt.scatter(self.target_position[0], self.target_position[1], label='Target', color='green', marker='o')
        plt.scatter(self.initial_position[0], self.initial_position[1], label='Start', color='pink', marker='o')
    
        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Optimization Steps for Trajectory with Fixed Distance')
        plt.show()

if __name__ == '__main__':
    rospy.init_node('optimizer') # , anonymous=True, disable_signals=True)
    robot = FrankaRobot()
    mpc = RobotController()
    mpc.loop()
    rospy.spin()


    