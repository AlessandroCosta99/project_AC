##! /usr/bin/env python3

import math
import random
import rospy
import time
import pandas as pd
import numpy as np
import message_filters
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt

#Optimizer
from scipy.optimize import Bounds
from scipy.spatial import distance
from scipy.optimize import minimize
from scipy.optimize import BFGS
from scipy.optimize import LinearConstraint
from scipy.optimize import NonlinearConstraint
from matplotlib.animation import FuncAnimation


## Pytorch
import torch
import torch.onnx
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

#models
#from ACTP.ACTP_model import ACTP


class RobotController():
    def __init__(self):
        self.stop = 0
        self.time_step = 0
        self.prev_time_step = 0
        self.tau = 0
        self.E = 0
        self.x0 = 1500
        self.y0 = 500
        self.x_f = 1400
        self.y_f = 300
        self.num_int_points = 10
        self.trajectory_history = []
        self.cost_history = []
        self.initial_position = np.array([self.x0, self.y0])
        self.target_position = np.array([self.x_f, self.y_f])
        #self.d = 2
        self.d = np.linalg.norm(self.target_position - self.initial_position) / (self.num_int_points+1)
        self.points = []

        #rospy.init_node('listener', anonymous=True, disable_signals=True)

        self.init_sub()
        # self.control_loop()
        self.compute_trajectory()
        self.opt_theta = self.gen_opt_traj()
        self.optimized_trajectory = self.circular_to_cartesian(self.opt_theta) 
        self.plot_trajectory()
        print(self.optimal_trajectory[1])

        
    def init_sub(self):
        rospy.init_node('optimizer', anonymous=True, disable_signals=True)
        self.optimal_traj_pub = rospy.Publisher('/optimal_traj', Float64MultiArray, queue_size=11)
        # robot_data = message_filters.Subscriber('/robot_state',Float64MultiArray, queue_size = 11)
        # self.robot_subscriber = [robot_data]
        # ts_sync = message_filters.ApproximateTimeSynchronizer(self.robot_subscriber, queue_size=1, slop=0.1, allow_headerless=True)
        # ts_sync.registerCallback(self.sub_cb)


    def cost_callback(self, theta_values):    #usefull for tracking the cost value during the optimization
        points = self.circular_to_cartesian(theta_values)
        cost = self.calculate_cost(points, self.target_position)
        self.trajectory_history.append(points.copy())
        self.cost_history.append(cost)
    
    def gen_opt_traj(self):
        # Initial guess for theta values
        #initial_guess = np.random.uniform(0, 2*np.pi, self.num_points - 1)
        initial_theta = np.zeros(self.num_int_points + 1)
        # Bounds for the theta values (individual bounds for each element)
        bounds = None
        # Run the optimization with the callback function
        result = minimize(self.obj_func, initial_theta, bounds=bounds, callback=self.cost_callback, method='BFGS')
        # Extract the optimized theta values
        optimal_theta = result.x
        return optimal_theta

    def obj_func(self, theta_values):
        self.points = self.circular_to_cartesian(theta_values)
        cost = self.calculate_cost(self.points, self.target_position)
        return cost
       
    def calculate_cost(self, points, target_position):
        return np.sum(np.linalg.norm(points - target_position, axis=1)**2)

    def circular_to_cartesian(self,theta_values):
        x = np.zeros(self.num_int_points+2)
        y = np.zeros(self.num_int_points+2)
        for i in range(self.num_int_points+2):
            if i == 0:
                x[i] = self.initial_position[0]
                y[i] = self.initial_position[1]
            else:
                x[i] = x[i-1] + self.d * np.cos(theta_values[i-1])
                y[i] = y[i-1] + self.d * np.sin(theta_values[i-1])
           
        return np.column_stack((x, y))

    def compute_trajectory(self):
        self.opt_theta = self.gen_opt_traj()
        self.optimal_trajectory = self.circular_to_cartesian(self.opt_theta) 

        traj_msg = Float64MultiArray()
        traj_msg.data = self.optimal_trajectory[1]
        self.optimal_traj_pub.publish(traj_msg)

    def control_loop(self):

        rate = rospy.Rate(50)
    
        while not rospy.is_shutdown():
            
            try:
                if self.time_step == 0:
                    self.t0 = time.time()

                if self.stop == 0.0 and self.time_step > 0 and self.time_step > self.prev_time_step:
                    self.compute_trajectory()
                elif self.stop == 1.0:
                    [sub.sub.unregister() for sub in self.robot_subscriber]
                    break

                rate.sleep()
            
            except KeyboardInterrupt:
                break

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
    io = RobotController()
    #io.animate_optimization()