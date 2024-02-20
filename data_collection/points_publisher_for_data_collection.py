#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import math

def pose_publisher():
    # Initialize the node with a name
    rospy.init_node('pose_publisher_node')
    
    # Create a Publisher object that will publish to the /pose_topic topic
    pose_pub = rospy.Publisher('/pose_topic', PoseStamped, queue_size=10)
    
    # Create a Rate object to control the publishing frequency
    rate = rospy.Rate(1001) #  10 Hz
    
    while not rospy.is_shutdown():
        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "panda_link0"
        pose_msg.pose.position.x =  0.60546
        pose_msg.pose.position.y =  -0.23
        pose_msg.pose.position.z =  0.80
        print(pose_msg)
        # Publish the message
        pose_pub.publish(pose_msg)
        
        # Sleep for the remaining time until the next cycle
        rate.sleep()

if __name__ == '__main__':
    pose_publisher()
