#! /usr/bin/env python3
from argparse import _ActionsContainer
from os import device_encoding
import rospy
import time
import pickle
import datetime
import numpy as np
import pandas as pd
import message_filters
from pickle import load
import numpy.matlib as mat
from numpy.random import seed
from pykalman import KalmanFilter
# from xela_server.msg import XStream
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Float64
from scipy.spatial.transform import Rotation as R


## Optimizer
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

# import onnx
# import onnxruntime

from ACTP.ACTP_model import ACTP
# from ClassifierLSTM.classifier_model import ClassifierLSTM
from ClassifierLSTM.seq_classifier_lstm import ClassifierLSTM

from scipy.special import expit
from scipy.misc import derivative

seed = 42
context_frames = 10
sequence_length = 20
torch.manual_seed(seed)
torch.backends.cudnn.benchmark = False
torch.backends.cudnn.deterministic = True
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class TrajectoryAdaptation():
    def __init__(self):
        self.stop = 0.0
		self.time_step = 0
		self.prev_time_step = 0
		self.translation_x = 0.35
		self.translation_y = 0.46
		# self.previous_w = np.random.normal(0, 0.1, 5)
		self.robot_data 	      = np.zeros((1000, 6))
		self.action_data 	      = np.zeros((1000, 6))
		self.robot_data_scaled 	  = np.zeros((1000, 6))
		self.action_data_scaled   = np.zeros((1000, 6))

		self.optimal_weights      = np.zeros((700, 18))
		self.optimal_trajectory   = np.zeros((700, 10, 6))
		self.opt_execution_time   = np.zeros(700)
		self.optimality           = np.zeros(700)
		self.num_itr              = np.zeros(700)
		self.constr_violation     = np.zeros(700)   ###not sure

        self.save_results_path = "/home/alessandro/tactile_control_ale/src/trajectory_adaptation/data/pre_testing"
        
        #node initialization
        rospy.init_node('listener', anonymous=True, disable_signals=True)
        #functions activation
		self.model_predict_001_init()
		self.load_scalers()
		self.init_sub()
		self.control_loop()

    def init_sub(self):
        sync_data_sub = message_filters.Subscriber('/sync_data', Float64MultiArray) #form sync_publisher_node.py
        self.sync_subscriber = [sync_data_sub]
        ts_sync = message_filters.ApproximateTimeSynchronizer(self.sync_subscriber, queue_size=1, slop=0.1, allow_headerless=True)
        ts_sync.registerCallback(self.sub_cb)
        self.slip_prediction_pub = rospy.Publisher('/slip_prediction', Float64, queue_size=11)
        self.optimal_traj_pub = rospy.Publisher('/optimal_traj', Float64MultiArray, queue_size=11)
    
    def load_scalers(self):
        self.scaler_path = "/home/alessandro/tactile_control_ale/src/trajectory_adaptation/python-scripts/scalers"
        #ROBOT SCALER
        #e.g. 		self.robot_min_max_scalar    = [load(open(self.scaler_path + '/robot_min_max_scalar_'+feature +'.pkl', 'rb')) for feature in ['vx', 'vy', 'vz', 'wx', 'wy', 'wz']]
        #think i will just have px and py feature, maximum pz
        #POSE PREDICTION SCALER

    def descale_data():
        #dont know if i need this
        return None
    
    def model_predict_001_init(self):
            self.model = torch.load("/home/alessandro/tactile_control_ale/src/trajectory_adaptation/models/ACTP", map_location='cpu').to(device).double()
            self.model.eval()

    def BasisFuncGauss(self, N, h, dt):
        self.T = int(round(1/dt+1))
        self.Phi = np.zeros((self.T-1,N))
        for z in range(0,self.T-1):
            t = z*dt
            self.phi = np.zeros((1, N))
            for k in range(1,N+1):
                c = (k-1)/(N-1)
                self.phi[0,k-1] = np.exp(-(t - c)*(t - c)/(2*h))
            self.Phi[z,:N] = self.phi[0, :N]
        self.Phi = self.Phi/np.transpose(mat.repmat(np.sum(self.Phi,axis=1),N,1)); #[TxN]   Normalizes the rows of self.Phi such that each row sums to 1.
        return self.Phi #[TxN]
    
    def gen_opt_traj(self, nom_traj):	
        self.nom_traj = nom_traj
        self.num_basis = 4
        self.Phi_mat = self.BasisFuncGauss(self.num_basis, 0.015, dt=0.1)
        self.phi_mat_augmented = np.tile(self.Phi_mat, (1, 6)) #create an augmented matrix by repeating the self.Phi_mat matrix horizontally 6 times
        w0 = np.random.normal(0, 0.1, 18)
	
        optimizer_options = {'verbose': 0, 'xtol':1e-08, 'gtol':1e-08,  'maxiter':20, 'disp':False}
        res = minimize(self.obj_func, w0, method='trust-constr', options=optimizer_options, jac=self.obj_jac)
		#obj_func specify teh jacobian(gradient) of the objective function
		
		# optimizer_options = {'maxiter':10, 'disp':True}
		# res = minimize(self.obj_func, w0, method='nelder-mead', options={'maxiter':10, 'disp':True})
		
        self.opt_execution_time[self.time_step] = res.execution_time
        self.optimality[self.time_step] = res.optimality
        self.num_itr[self.time_step] = res.nit
        self.constr_violation[self.time_step] = res.constr_violation
		
        return res.x


    def obj_func(self, w):
            ref_point = 
            return np.mean((np.matmul(self.phi_mat_augmented,w)-ref_point)**2)
    
    def scale_action(self, action):
        for index, min_max_scalar in enumerate(self.robot_min_max_scalar):
            action[:, index] = np.squeeze(min_max_scalar.transform(action[:, index].reshape(-1, 1)))

        action = action[:, np.newaxis, :].astype(np.float32)

        return action
    
    

    def compute_trajectory(self):
        position_seq =  
        robot_seq  = self.robot_data[self.time_step-10 : self.time_step , : ]
        action_seq = self.action_data[self.time_step-10 : self.time_step , : ] # shpe: 10x6 <-- i want just 2 or 1 dof

        self.optimal_weight = self.gen_opt_traj(action_seq)
        self.optimal_weights[self.time_step] = self.optimal_weight

        x = 1
        y = 1

        self.ref_trajectory[self.time_step, :, 0] = x
        self.ref_trajectory[self.time_step, :, 1] = y

        x += np.matmul(self.Phi_mat, self. optimal_weight[:3])
        y += np.matmul(self.Phi_mat, self. optimal_weight[3:6])

    def print_rate(self):
            t1 = time.time()
            rate = self.time_step / (t1-self.t0)
            print("RATE: ", rate)


if __name__ == "__main__":	
	mf = TrajectoryAdaptation()

	mf.print_rate()
	mf.save_results()