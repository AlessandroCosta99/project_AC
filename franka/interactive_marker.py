#!/usr/bin/env python

import rospy 
import numpy as np
import tf
 
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

# franka_init_state = None
franka_init_state = None

def franka_state_callback(msg):
    initial_quaternion = \
        tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
    franka_init_state.pose.orientation.x = initial_quaternion[0]
    franka_init_state.pose.orientation.y = initial_quaternion[1]
    franka_init_state.pose.orientation.z = initial_quaternion[2]
    franka_init_state.pose.orientation.w = initial_quaternion[3]
 
if __name__ == "__main__":
    rospy.init_node("equilibrium_pose_node")
    state_sub = rospy.Subscriber("franka_state_controller/franka_states",
                                 FrankaState, franka_state_callback)
    pose_pub = rospy.Publisher(
        "target_pose", PoseStamped, queue_size=10)
    
    rate = rospy.Rate(1001)
    while not rospy.is_shutdown():

        if franka_init_state != None:

            msg = PoseStamped() 
            msg.header.frame_id = "panda_link0"
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = 0.35234
            msg.pose.position.x = 0.0000
            msg.pose.position.x = 0.5523
            msg.pose.orientation.x = franka_init_state.pose.orientation.x
            msg.pose.orientation.y = franka_init_state.pose.orientation.y
            msg.pose.orientation.z = franka_init_state.pose.orientation.z
            msg.pose.orientation.w = franka_init_state.pose.orientation.w

                
            pose_pub.publish(msg)


    rospy.spin()
