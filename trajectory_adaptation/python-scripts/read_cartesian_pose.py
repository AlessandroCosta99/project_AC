import os
import sys
import time
import math
import rospy
import random
import datetime
import message_filters
import moveit_msgs.msg
import moveit_commander
import matplotlib.pyplot

import numpy as np
import pandas as pd

from matplotlib.pyplot import imsave
from math import pi
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool, Int16MultiArray, Float32MultiArray
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState, Image
from moveit_msgs.msg import CollisionObject, DisplayTrajectory, MotionPlanRequest, Constraints, PositionConstraint, JointConstraint
from geometry_msgs.msg import PoseStamped, Pose, Point
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_commander.conversions import pose_to_list

rospy.init_node('read_pose', anonymous = True)
ee_state_vec= []
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningScene()
move_group = moveit_commander.MoveGroupCommander("panda_arm")
coord_pub = rospy.Publisher('/cartesian_pose', Pose ,queue_size=1000)
rate = rospy.Rate(2)
while not rospy.is_shutdown():
    ee_state=move_group.get_current_pose().pose
    ee_state_vec.append(ee_state)
    coord_pub.publish(ee_state)
    rate.sleep()
print(ee_state_vec)


