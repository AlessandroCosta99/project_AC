#!/usr/bin/env python3
# Author: ALESSANDRO COSTA 02/24
import time
import torch
import rospy
import joblib
import numpy as np
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import Point
import message_filters
#from collection import dequeue


#Models
from rt_model.rt_model import SequencePredictor

#input: b_t, x_t, x_t-1, v_t, v_t-1
#output: b_t+1

class RobotReader(object):
    def __init__(self):
        # ROS node
        rospy.init_node('strawberry_prediction', disable_signals=True)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.time_step = 0
        self.stop = 0.0
        self.pred_horizon = 1

        # Publishers
        self.pub_predicted_positions = rospy.Publisher("/predicted_strawberry_positions", Float64MultiArray, queue_size=10)

        # Placeholders for received data
        self.berry_position      = np.zeros((500,2)).astype(np.float32)
        self.robot_ee_pose       = np.zeros((500,2)).astype(np.float32)
        self.robot_action        = np.zeros((self.pred_horizon,2)).astype(np.float32)
        self.action_concat		 = np.zeros((500, x, y)).astype(np.float32)   ##!!!!!!!CHECK DIMENSION-according to scalers

        #activate methods
        self.init_sub()
        self.load_model()
        self.load_scalers()
        self.control_loop()

    def init_sub(self):
        self.sub_strawberry = message_filters.Subscriber("/strawberry_position", Point)
        self.sub_robot_ee = message_filters.Subscriber("/franka_state_controller/franka_states", FrankaState)#, self.franka_state_callback)
        self.robot_action_sub = message_filters.Subscriber('/candidate_action', Float64MultiArray)
        sync_sub = [self.sub_strawberry, self.sub_robot_ee, self.robot_action_sub]
        sync_cb = message_filters.ApproximateTimeSynchronizer(sync_sub,  10, 0.1, allow_headerless=True) 
        sync_cb.registerCallback(self.callback)

    def load_model(self):
        model_path = "/home/alessandro/tactile_control_ale_ws/src/trajectory_adaptation/python-scripts/..."
        self.model = SequencePredictor()
        self.model.load_state_dict(torch.load(model_path + '/model.pth', map_location=self.device))
        self.model.eval()

    def load_scalers(self):
        scaler_path = "/home/alessandro/tactile_control_ale_ws/src/trajectory_adaptation/python-scripts/scalers"
        self.berry_scaler = joblib.load(scaler_path + '/berry_scaler.pkl')
        self.robot_scaler = joblib.load(scaler_path + '/robot_scaler.pkl')
        #self.action_scaler = joblib.load(scaler_path + '/action_scaler.pkl)

    def callback(self, berry_pose_msg, robot_pose_msg, robot_action_msg):
        self.robot_ee_pose[self.time_step] = np.array([robot_pose_msg.data[13], robot_pose_msg.data[14]])
        self.berry_position[self.time_step] = np.array([berry_pose_msg.x, berry_pose_msg.y])
        self.robot_action = robot_action_msg.data

        if self.time_step > self.pred_horizon: #if not we don't have enough history data to feed the model with  
            #-Scaling
            scaled_robot = np.zeros_like(self.robot_ee_pose[self.time_step-self.pred_horizon:self.time_step, :]).astype(np.float32)  #prediction horizon = 1 in this case
            #the robot actions came from the mpc (optimizer) so the first i recive already represent the cadidate self.prediction_horizion evolution of robot state
            scaled_action = np.zeros_like(self.robot_action[self.time_step]).astype(np.float32)

            scaled_robot = self.robot_scaler.transform(self.robot_ee_pose[self.time_step])
            scaled_action = self.robot_scaler.transform(self.robot_action)
            #Concatenate, past data, present data and future data (action)
            scaled_action_r = np.concatenate((scaled_robot, scaled_action), axis=0).astype(np.float32)
            self.action_concat[self.time_step] = scaled_action_r
            
            strawberry_pose_scaled = np.zeros_like(self.berry_position[self.time_step])  #input only b_t
            strawberry_pose_scaled = self.berry_scaler.transform(self.berry_position)
            
            self.scaled_action = torch.from_numpy(self.action_concat[self.time_step]).unsqueeze(1)  #unsqueeze is like transpose a vector into a column
            self.scaled_berry = torch.from_numpy(strawberry_pose_scaled).unsqueeze(1)

        self.time_step +=1

    def position_prediction(self):
        with torch.no_grad():
            predicted_positions = self.model(self.scaled_action, self.scaled_berry)
        return predicted_positions


    def publish_pred_berry_position(self):

        berry_pred_pos = self.position_prediction()

        # Scale Back if needed
        berry_pred_pos_np = berry_pred_pos.squeeze(0).cpu().numpy()
        #print(predicted_positions_np)
        berry_pred_pos_original = self.berry_scaler.inverse_transform(berry_pred_pos_np)
        #print(predicted_positions_original)

        berry_pred_msg = Float64MultiArray()

        berry_pred_msg.data = [berry_pred_pos_original]
    def save_data(self):
        np.save(self.save_path + "action.npy", self.robot_action[:self.time_step-1])
        np.save(self.save_path + "robot_pose.npy", self.robot_pose_data[:self.time_step-1]) # columns: x, y, z, eu_x, eu_y, eu_z, quat_x, quat_y, quat_z, _quat_w
        np.save(self.save_path + "action_scaled.npy", self.action_data_scaled[:self.time_step-1])
        np.save(self.save_path + "robot_data_scaled.npy", self.robot_data_scaled[:self.time_step-1])
	

    def control_loop(self):
        rate = rospy.Rate(60)
		
        while not rospy.is_shutdown():
			
            try:
                if self.time_step == 0:
                    self.t0 = time.time()

                if self.stop == 0.0 and self.time_step > self.pred_horizon+1:  #CHECK HERE
                    self.publish_pred_berry_position()
                elif self.stop == 1.0:
                    [sub.sub.unregister() for sub in self.sync_sub]
                    break

                rate.sleep()
			
            except KeyboardInterrupt:
                break
	
if __name__ == '__main__':
    robot_reader = RobotReader()