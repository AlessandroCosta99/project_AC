#!/usr/bin/env python

import rospy 
import numpy as np
 
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

franka_init_state = None
 
 
if __name__ == "__main__":
    rospy.init_node("equilibrium_pose_node")
   
    pose_pub = rospy.Publisher(
        "equilibrium_pose", PoseStamped, queue_size=10)
    
    while not rospy.is_shutdown():

        if franka_init_state != None:

            #
            franka_init_state.

            msg = PoseStamped() 
            msg.header.framke_id = "panda_link0"
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = x
            msg.pose.position.x = y
            msg.pose.position.x = z

            pose_pub.publish(msg)


     
    rospy.spin()
