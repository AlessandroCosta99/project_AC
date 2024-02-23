#!/usr/bin/env python3

import os
import sys
import time
import rospy
import random
import datetime
import message_filters
import moveit_msgs.msg
import moveit_commander

import numpy as np
import pandas as pd

from cv_bridge import CvBridge
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point
from franka_msgs.msg import FrankaState

#data collection for pushuing a single strawberry

class FrankaRobot(object):
    def __init__(self):
        super(FrankaRobot, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('FrankaRobotWorkshop', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.move_group.set_end_effector_link("panda_link8")
        self.group_names = self.robot.get_group_names()
        self.bridge = CvBridge()
        self.move_group.set_planning_pipeline_id("pilz_industrial_motion_planner") 
        self.move_group.set_planner_id("LIN")                      # Set to be the straight line planner
        print(self.move_group.get_interface_description().name)    # Print the planner being used.

        # scaling down velocity
        self.move_group.set_max_velocity_scaling_factor(0.1)      
        self.move_group.set_max_acceleration_scaling_factor(0.1)

        self.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"] 
        self.joint_home = [0.18984723979820606, -0.977749801850428, -0.22761550468348588, -2.526835711730154, -0.20211957115956533, 3.1466847225824988, 0.7832720796780586]
        self.joint_via_point = [-0.02785505427751484, -0.8694569696878132, -0.2636310946187023, -2.5146245994233247, -0.00886475483520896, 3.2627861557307716, 0.6308086736957446]
        self.joint_push = [0.015010430408195849, -0.3274054828968975, -0.07055645977825595, -1.9842514984624748, -0.06401155534892813, 3.2390935163497927, 0.8254441331095018]
        self.resets           = 5
        self.pushes_per_reset = 10

        #DATA SAVING:
        # Initialize the variable
        self.datasave_folder = "/home/alessandro/Dataset/new_dataset"

        self.robot_joint_sub = message_filters.Subscriber('/joint_states', JointState)
        self.robot_sub       = message_filters.Subscriber('/franka_state_controller/franka_states', FrankaState)
        self.centroids_sub   = message_filters.Subscriber('/strawberry_position', Point)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.robot_joint_sub, self.robot_sub, self.centroids_sub] , queue_size=1, slop=0.1, allow_headerless=True)


    def pushing_actions(self):
            
        total_pushes = 0
        failure_cases = 0
        for j in range(self.resets):
            for i in range(self.pushes_per_reset):
                print("  ")
                print("resets: {},   push: {},  total_pushes: {},   failure cases: {}".format(j, i, total_pushes, failure_cases))
                # self.go_via_point()
                if(j==0 and i==0):
                    self.go_home()
                self.go_push()

                #  Move to random target position:
                self.move_group.set_planner_id("LIN")

                target_pose = self.move_group.get_current_pose().pose
                target_pose.position.x = random.uniform(0.3,0.6)
                target_pose.position.y = random.uniform(-0.28,-0.15)
                #target_pose.position.z = 0
                # target_pose.orientation.x = -0.6346264749672597
                # target_pose.orientation.y =  0.2821901217833465
                # target_pose.orientation.z = -0.6675940185546088
                # target_pose.orientation.w = 0.2682092444876552
                target = self.move_group.set_pose_target(target_pose) 
                trajectory = self.move_group.plan(target)
                if trajectory[0] == False:
                    self.go_home()
                    failure_cases += 1
                    break

                time_for_trajectory = float(str(trajectory[1].joint_trajectory.points[-1].time_from_start.secs) + "." +str(trajectory[1].joint_trajectory.points[-1].time_from_start.nsecs))
                self.move_group.go(target, wait=False)
                self.data_saver(time_for_trajectory)
                

            total_pushes += 1
            
            self.go_via_point()
            
        self.go_push()

    def data_saver(self, time_for_trajectory):
        rate                = rospy.Rate(60)
        self.robot_states   = []
        self.franka_states = []
        self.times = []
        self.centroids  = []
        self.O_T_EE = []
        self.tau_J = []
        self.prev_i, self.i = 0, 1
        self.ts.registerCallback(self.read_robot_data)
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < time_for_trajectory:
            print(time_for_trajectory, "    ----     ", time.time() - t0, end="\r")
            self.i += 1
            rate.sleep()
        t1 = time.time()
        self.rate = (len(self.robot_states)) / (t1-t0)
        self.save_data()
        print("saved data")

    def read_robot_data(self, robot_joint_data, robot_state, centroids):
            if self.i != self.prev_i:
                self.prev_i = self.i

                self.tau_J = robot_state.tau_J
                self.tau_list = np.asarray(self.tau_J).tolist()
                self.O_T_EE = robot_state.O_T_EE
                self.pos_list = np.asarray(self.O_T_EE).tolist()
                self.times.append(robot_state.time)

                ee_state = self.move_group.get_current_pose().pose
                self.centroids.append([centroids.x, centroids.y])
                self.robot_states.append([robot_joint_data, ee_state])
                self.franka_states.append(self.tau_list + self.pos_list)

    def format_data_for_saving(self):
        self.robot_states_formated = []
        
        for data_sample_index in range(len(self.robot_states)):
            robot_joint_data = self.robot_states[data_sample_index][0]
            ee_state = self.robot_states[data_sample_index][1]
            self.robot_states_formated.append(list(robot_joint_data.position) + list(robot_joint_data.velocity) + list(robot_joint_data.effort) + 
                                            [ee_state.position.x, ee_state.position.y, ee_state.position.z,
                                            ee_state.orientation.x, ee_state.orientation.y, ee_state.orientation.z, ee_state.orientation.w])

    def save_data(self):


        #create new folder for this experiment:
        folder = str(self.datasave_folder + '/data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        mydir = os.mkdir(folder)

        self.format_data_for_saving()
        T0 = pd.DataFrame(self.robot_states_formated)
        T1 = pd.DataFrame(self.centroids)
        T2 = pd.DataFrame(self.franka_states)
        print(T0)
        robot_states_col = ["position_panda_joint1", "position_panda_joint2", "position_panda_joint3", "position_panda_joint4", "position_panda_joint5", "position_panda_joint6", "position_panda_joint7", "position_panda_finger_joint1", "position_panda_finger_joint2",
        "velocity_panda_joint1", "velocity_panda_joint2", "velocity_panda_joint3", "velocity_panda_joint4", "velocity_panda_joint5", "velocity_panda_joint6", "velocity_panda_joint7", "velocity_panda_finger_joint1", "velocity_panda_finger_joint2",
        "effort_panda_joint1", "panda_joint2", "effort_panda_joint3", "effort_panda_joint4", "panda_joint5", "effort_panda_joint6", "effort_panda_joint7", "effort_panda_finger_joint1", "effort_panda_finger_joint2",
        "ee_state_position_x", "ee_state_position_y", "ee_state_position_z", "ee_state_orientation_x", "ee_state_orientation_y", "ee_state_orientation_z", "ee_state_orientation_w"]
        franka_states_col = ["tau_0","tau_1", "tau_2", "tau_3", "tau_4", "tau_5", "tau_6", "O_T_EE1", "O_T_EE2", "O_T_EE3", "O_T_EE4", "O_T_EE5", "O_T_EE6", "O_T_EE7", "O_T_EE8", "O_T_EE9", "O_T_EE10", "O_T_EE11", "O_T_EE12", "O_T_EE13", "O_T_EE14", "O_T_EE15", "O_T_EE16"]


        T0.to_csv(folder + '/robot_state.csv', header=robot_states_col, index=False)
        T1.to_csv(folder + '/strawberry_position.csv', header=["berry_x", "berry_y"], index=False)
        T2.to_csv(folder + '/franka_states.csv', header=franka_states_col, index=False)


    def go_home(self):
        self.move_group.set_planner_id("PTP")         # Set to be the straight line planner
        self.move_group.go(self.joint_home, wait=True)
        
    def go_push(self):
        self.move_group.set_planner_id("PTP")
        # self.move_group.go(self.target_pose, wait=True)
        self.move_group.go(self.joint_push, wait=True)

    def go_via_point(self):
        self.move_group.set_planner_id("LIN")
        self.move_group.go(self.joint_via_point, wait=True)

    def get_robot_task_state(self):
        robot_ee_pose = self.move_group.get_current_pose().pose
        return [robot_ee_pose.position.x, robot_ee_pose.position.y, robot_ee_pose.position.z], [robot_ee_pose.orientation.x, robot_ee_pose.orientation.y, robot_ee_pose.orientation.z, robot_ee_pose.orientation.w]

    def create_pose(self, start_position, orientation):
        pose = PoseStamped()
        pose.header.frame_id = '/panda_link0'
        pose.pose.position.x = start_position[0]
        pose.pose.position.y = start_position[1]
        pose.pose.position.z = start_position[2]

        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        return pose

   


if __name__ == '__main__':
    robot = FrankaRobot()
    robot.pushing_actions()