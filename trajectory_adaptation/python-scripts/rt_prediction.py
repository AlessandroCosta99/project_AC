#!/usr/bin/env python3
# Author: OUSSAMA LAZAZAT 02/24

import torch
import rospy
import joblib
import numpy as np
from geometry_msgs.msg import Pose
from model import SequencePredictor
from std_msgs.msg import Float64MultiArray

class RobotReader(object):
    def __init__(self):
        # ROS node
        rospy.init_node('strawberry_prediction')

        # Model and scaler
        self.model = SequencePredictor(4, 50, 20, 2)
        self.model.load_state_dict(torch.load('model.pth', map_location=torch.device('cpu')))
        self.distance_scaler = joblib.load('scaler.pkl')
        self.model.eval()

        # Subscribers
        self.sub_strawberry = rospy.Subscriber("/strawberry_positions", Float64MultiArray, self.callback_strawberry)
        self.sub_robot_ee = rospy.Subscriber("/robot_ee_state", Float64MultiArray, self.callback_robot_ee)

        # Publishers
        self.pub_predicted_positions = rospy.Publisher("/predicted_strawberry_positions", Float64MultiArray, queue_size=10)

        # Placeholders for received data
        self.strawberry_positions = None
        self.robot_ee_positions = None

    def callback_strawberry(self, data):
        self.strawberry_positions = np.array(data.data).reshape(-1, 2)

    def callback_robot_ee(self, data):
        self.robot_ee_positions = np.array(data.data).reshape(-1, 2)

    def predict_and_publish(self):
        # Check if data is available
        if self.strawberry_positions is not None and self.robot_ee_positions is not None:
            # Combine data
            combined_data = np.hstack((self.strawberry_positions, self.robot_ee_positions))
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
            self.robot_ee_positions = None

if __name__ == '__main__':
    robot_reader = RobotReader()
    rate = rospy.Rate(500)  # 500 Hz <------------------------------------- !!!!!!!!!!! ???????????????
    while not rospy.is_shutdown():
        robot_reader.predict_and_publish()
        rate.sleep()