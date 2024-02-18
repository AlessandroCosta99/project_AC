#!/usr/bin/env python3
# Author: OUSSAMA LAZAZAT 02/24

import torch
import rospy
import joblib
import numpy as np
from std_msgs.msg import Float64MultiArray
from rt_model.rt_model import SequencePredictor
from franka_msgs.msg import FrankaState

class RobotReader(object):
    def __init__(self):
        # ROS node
        rospy.init_node('strawberry_prediction')
        self.robot_pose = []
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
        self.sub_strawberry = rospy.Subscriber("/strawberry_positions", Float64MultiArray, self.callback_strawberry)
        self.sub_robot_ee = rospy.Subscriber("/franka_state_controller/franka_states", Float64MultiArray, self.callback_robot_ee)

        # Publishers
        self.pub_predicted_positions = rospy.Publisher("/predicted_strawberry_positions", Float64MultiArray, queue_size=10)

        # Placeholders for received data
        self.strawberry_positions = None
        self.robot_pose = None

    def callback_strawberry(self, data):
        self.strawberry_positions = np.array(data.data).reshape(-1, 2)

    def franka_state_callback(self, msg):
        self.robot_pose[0] = msg.O_T_EE[12]  # x
        self.robot_pose[1] = msg.O_T_EE[13]  # y 


    def predict_and_publish(self):
        # Check if data is available
        if self.strawberry_positions is not None and self.robot_pose is not None:
            # Combine data
            combined_data = np.hstack((self.strawberry_positions, self.robot_pose))
            combined_data_scaled = self.distance_scaler.transform(combined_data)

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
            self.strawberry_positions = None
            self.robot_pose = None

if __name__ == '__main__':
    robot_reader = RobotReader()
    rate = rospy.Rate(50)  # 50 Hz <------------------------------------- !!!!!!!!!!! ???????????????
    while not rospy.is_shutdown():
        robot_reader.predict_and_publish()
        rate.sleep()