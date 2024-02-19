#!/usr/bin/env python3
# Author: OUSSAMA LAZAZAT 02/24

import torch
import rospy
import joblib
import numpy as np
from std_msgs.msg import Float64MultiArray
from rt_model.rt_model import SequencePredictor
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import Point
import message_filters
from collections import deque

class RobotReader(object):
    def __init__(self):
        # ROS node
        rospy.init_node('strawberry_prediction')
        self.robot_pose = [0.0] * 2
        input_size = 4
        hidden_size = 50
        output_size = 20
        num_layers = 2
        model_path = "/home/alessandro/tactile_control_ale_ws/src/trajectory_adaptation/python-scripts"
        scaler_path = "/home/alessandro/tactile_control_ale_ws/src/trajectory_adaptation/python-scripts/scalers"
        # Model and scaler
        self.model = SequencePredictor(input_size, hidden_size, output_size, num_layers)
        self.model.load_state_dict(torch.load(model_path + '/model.pth', map_location=torch.device('cpu')))
        self.distance_scaler = joblib.load(scaler_path + '/scaler.pkl')
        self.model.eval()

        # Subscribers
        self.sub_strawberry = message_filters.Subscriber("/strawberry_position", Point)#, self.callback_strawberry)
        self.sub_robot_ee = message_filters.Subscriber("/franka_state_controller/franka_states", FrankaState)#, self.franka_state_callback)
        sync_sub = [self.sub_strawberry, self.sub_robot_ee]
        sync_cb = message_filters.ApproximateTimeSynchronizer(sync_sub,  10, 0.1, allow_headerless=True) 
        sync_cb.registerCallback(self.callback)

        # Publishers
        self.pub_predicted_positions = rospy.Publisher("/predicted_strawberry_positions", Float64MultiArray, queue_size=10)

        # Placeholders for received data
        self.strawberry_positions = deque(maxlen=20)
        self.robot_ee_pose = deque(maxlen=20)
        self.index = 0

    def callback(self, berry_pose, robot_pose):
        
        self.strawberry_positions.append(berry_pose.x)
        self.strawberry_positions.append(berry_pose.y)

        self.robot_ee_pose.append(robot_pose.O_T_EE[12])  # x
        self.robot_ee_pose.append(robot_pose.O_T_EE[13])  # y 



    def predict_and_publish(self):

        # Check if data is available and of the right shape
        #if self.strawberry_positions is not None and self.robot_pose is not None:
            # Combine data
        if len(self.strawberry_positions) == 20 and len(self.robot_ee_pose) == 20:
            
            b = np.array(self.strawberry_positions)
            berries = b.reshape((10,2))
            r= np.array(self.robot_ee_pose)
            robot = b.reshape((10,2))
            robot_pose_scaled = self.distance_scaler.transform(robot)
            strawberry_pose_scaled = self.distance_scaler.transform(berries)
            combined_data_scaled = np.hstack((robot_pose_scaled, strawberry_pose_scaled))

            # Convert to torch tensor
            input_tensor = torch.FloatTensor(combined_data_scaled).unsqueeze(0)

            # Prediction
            with torch.no_grad():
                predicted_positions = self.model(input_tensor)

            # Scale bback
            predicted_positions_np = predicted_positions.squeeze(0).cpu().numpy()
            predicted_positions_original = self.distance_scaler.inverse_transform(predicted_positions_np)
            
            
            # Publisher
            msg = Float64MultiArray()
            msg.data = predicted_positions_original.flatten()
            self.pub_predicted_positions.publish(msg)

            # Reset placeholders

if __name__ == '__main__':
    robot_reader = RobotReader()
    rate = rospy.Rate(50)  # 50 Hz <------------------------------------- !!!!!!!!!!! ???????????????
    while not rospy.is_shutdown():
        robot_reader.predict_and_publish()
        rate.sleep()