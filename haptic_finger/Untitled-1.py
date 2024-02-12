#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

rospy.init_node('coordinates_publisher', anonymous=True)

pub = rospy.Publisher('/next_pose', Float64MultiArray, queue_size=10)

# Define the initial coordinates
x =  0.0
y =  0.0
vec = [x,y]

# Define a rate at which to publish the messages
rate = rospy.Rate(1)  #  1 Hz

while not rospy.is_shutdown():
    # Create a PointStamped message
    point_stamped = Float64MultiArray()
    point_stamped.data = vec


    # Publish the message
    pub.publish(point_stamped)

    # Increment the coordinates
    vec[0] +=  1
    vec[1] +=  1

    # Sleep for the desired rate
    rate.sleep()