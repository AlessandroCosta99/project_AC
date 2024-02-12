#!/usr/bin/env python3

import os
import sys
import cv2
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
from std_msgs.msg import String, Bool, Int16MultiArray
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState, Image
from moveit_msgs.msg import CollisionObject, DisplayTrajectory, MotionPlanRequest, Constraints, PositionConstraint, JointConstraint
from geometry_msgs.msg import PoseStamped, Pose, Point
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_commander.conversions import pose_to_list

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
        # self.move_group.set_planner_id("LIN")                      # Set to be the straight line planner
        print(self.move_group.get_interface_description().name)    # Print the planner being used.

        # scaling down velocity
        
        self.move_group.set_max_velocity_scaling_factor(0.5)      
        self.move_group.set_max_acceleration_scaling_factor(0.5)

        self.pushing_z_height        = 0.15
        self.starting_depth          = 0.30 # was 35
        self.finish_depth            = 0.40
        self.starting_position_width = [-0.08, 0.3]
        self.starting_position = -0.08
        self.finish_position = 0.3
        self.pushing_orientation     = [-0.9238638957839016, 0.3827149349905697, -0.0020559535525728366, 0.0007440814108405214]

        self.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"] 
        self.joint_home = [0.18984723979820606, -0.977749801850428, -0.22761550468348588, -2.526835711730154, -0.20211957115956533, 3.1466847225824988, 0.7832720796780586]
        # self.joint_via_point = [0.04607601949356312, 0.17021933704864067, -0.13281828155595027, -1.3124015096343356, -0.019060995645703918, 3.009036840939284, 0.8235806091527144]
        self.joint_via_point = [-0.2827285690477829, -0.10776317482340839, 0.25531004340188546, -1.720866153466074, 0.49796132276751903, 3.2227085454915225, 0.20333962609574804]
        # self.joint_push = [0.04607601949356312, 0.17021933704864067, -0.13281828155595027, -1.3124015096343356, -0.019060995645703918, 3.009036840939284, 0.8235806091527144]
        # self.joint_push = [0.04308905999495077, 0.1585832643771773, -0.14940384170854504, -1.3571466245316623, 0.02975108286944235, 3.0149748432071917, 0.7491484249470901]
        self.joint_push = [0.04446678416070445, 0.1751057482563725, -0.16586388862760443, -1.3552765184408364, 0.0260995250113227, 3.0163658870067884, 0.7461208454668522]
        self.resets           = 3
        self.pushes_per_reset = 3

        #DATA SAVING:
        # Initialize the variable
        pat = None
        # Keep asking the user until a correct choice is made
        while pat is None:
            user_choice = input("Testing pipeline (t) or actual sampling (a)? ")

            if user_choice == 't':
                pat = 1
                self.datasave_folder = "/home/alessandro/Dataset/localization/preliminary_tries"
            elif user_choice == 'a':
                pat = 1
                self.datasave_folder = "/home/alessandro/Dataset/Pushing_Single_Strawberry/second_collection"
            else:
                 print("Invalid choice. Please choose 't' or 'a'.")
        # self.datasave_folder = "/home/alessandro/Dataset/Pushing_Single_Strawberry/first_collection"
        self.robot_sub       = message_filters.Subscriber('/joint_states', JointState)
        # self.fing_cam_sub = message_filters.Subscriber("/fing_camera/color/image_raw", Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.robot_sub] , queue_size=1, slop=0.1, allow_headerless=True)
        ######################################################################, self.fing_cam_sub

    def pushing_actions(self):
        start_position, start_ori = self.get_robot_task_state()
        
        
        # Initialize the variable
        tra = None
        # Keep asking the user until a correct choice is made
        while tra is None:
            user_choice = input("Choose 'l' or 'c': ")

            if user_choice == 'l':
                tra = 1
            elif user_choice == 'c':
                tra = 2
            else:
                 print("Invalid choice. Please choose 'l' or 'c'.")
         
            
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

                #LINEAR MOTION
                if tra == 1:
                    # 1. Move to random start position:
                    self.move_group.set_planner_id("LIN")                      # Set to be the straight line planner
                    # start_y_position = random.uniform(self.starting_position_width[0], self.starting_position_width[1])
                    start_y_position = self.starting_position
                    start_pose = self.move_group.get_current_pose().pose
                    start_pose.position.x = self.starting_depth
                    # start_pose.position.x = random.uniform(self.starting_depth, self.finish_depth)
                    start_pose.position.y = start_y_position
                    start_pose.position.z = self.pushing_z_height
                    start_pose.orientation.x = self.pushing_orientation[0]
                    start_pose.orientation.y = self.pushing_orientation[1]
                    start_pose.orientation.z = self.pushing_orientation[2]
                    start_pose.orientation.w = self.pushing_orientation[3]
           
                    target = self.move_group.set_pose_target(start_pose) ###### the problem is here
                    print("ci siamo qui?", target) 
                    trajectory = self.move_group.plan(target)    
                    print(trajectory[0])
                    if trajectory[0] == False:
                        self.go_home()
                        failure_cases += 1
                        break
                    self.move_group.go(target, wait=True)
                    

                    #2. Execute pushing action to second random position:
                    self.move_group.set_planner_id("LIN")                      # Set to be the straight line planner
                    start_pose = self.move_group.get_current_pose().pose
                    # finish_y_position = random.uniform(self.starting_position_width[0], self.starting_position_width[1])
                    finish_y_position = self.finish_position
                    finish_x_position = random.uniform(self.starting_depth, self.finish_depth)
                    # finish_y_position = random.uniform(self.starting_position_width[0], start_y_position)
                    
                    start_position, start_ori = self.get_robot_task_state()    # calculate required angle of end effector:
                    euler_ori = euler_from_quaternion(start_ori)
                    euler_ori = list(euler_ori)
                    rotational_change = math.atan((start_y_position - finish_y_position) / (self.finish_depth - self.starting_depth))
                    # rotational_change = math.atan((start_y_position - finish_y_position) / (self.starting_depth - start_pose.position.x))
                    # rotational_change = math.atan((start_y_position - finish_y_position) / (finish_x_position - self.starting_depth))
                    # rotational_change = math.atan((start_y_position - finish_y_position) / (finish_x_position - start_pose.position.x))
                    print(f"rotational change: {rotational_change}")
                    euler_ori[2] += rotational_change
                    joint_goal = self.move_group.get_current_joint_values()
                    # joint_goal[-1] += rotational_change - pi/2
                    # joint_goal[-1] += - pi
                    self.move_group.go(joint_goal, wait=True)
                    self.move_group.stop()
                    

                    a = input("start_collecting data - enter to continue:")
                    if a == "n" or a == "N":
                        break


                    #LINEAR PUSHING IN ONE DIRECTION

                    # 3. Make pushing action:
                    self.move_group.set_planner_id("LIN")
                    #finish_y_position += 0.1
                    _, final_ori_quat = self.get_robot_task_state()
                    finish_pose = self.move_group.get_current_pose().pose
                    # finish_pose.position.x = finish_pose.position.x - 0.2
                    finish_pose.position.x -= 0.0
                    finish_pose.position.y -= 0.12
                    # finish_pose.position.z += 0.05
                    finish_pose.orientation.x = final_ori_quat[0]
                    finish_pose.orientation.y = final_ori_quat[1]
                    finish_pose.orientation.z = final_ori_quat[2]
                    finish_pose.orientation.w = final_ori_quat[3]
                    target = self.move_group.set_pose_target(finish_pose)
                    trajectory = self.move_group.plan(target)
                    if trajectory[0] == False:
                        print("False")
                        self.go_home()
                        failure_cases += 1
                        break
                    time_for_trajectory = float(str(trajectory[1].joint_trajectory.points[-1].time_from_start.secs) + "." +str(trajectory[1].joint_trajectory.points[-1].time_from_start.nsecs))

                    self.move_group.go(target, wait=False)
                    self.data_saver(time_for_trajectory)
                
                #ARC MOTION
                elif tra == 2:
                    #ARC MOTION
            

                    pilz_pose = MotionPlanRequest()
                    pilz_pose.planner_id = "CIRC"
                    pilz_pose.group_name = "panda_arm"
                    pilz_pose.max_velocity_scaling_factor =  0.2
                    print("max vel:",pilz_pose.max_velocity_scaling_factor )
                    pilz_pose.max_acceleration_scaling_factor = 0.05
                    pilz_pose.start_state.joint_state.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"] 
                    pilz_pose.start_state.joint_state.position = self.move_group.get_current_joint_values()
                    # pilz_pose.start_state.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    # pilz_pose.start_state.joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    pilz_pose.start_state.joint_state.header.stamp = rospy.Time.now()
            
                    pose = self.move_group.get_current_pose()
                    # print(pose)

                    #----------RANDOM------------
                    # pose.pose.position.z += random.uniform(0.02,0.06)
                    # pose.pose.position.y -= random.uniform(0.08,0.16)
                    # pose.pose.position.x -= random.uniform(-0.04,0.05)*1.5
                    # pose.pose.orientation.x = random.uniform(0.6,0.7)
                    # pose.pose.orientation.y = random.uniform(-0.1,0.1)
                    # pose.pose.orientation.z = random.uniform(0.65,0.75)    
                    # pose.pose.orientation.w = random.uniform(0,0.1)
                    pose.pose.position.z += 0.00  # RIGHT ONE DX !!!!
                    pose.pose.position.y -= 0.2  # RIGHT ONE DX !!!!
                    pose.pose.position.x -= 0.00  # RIGHT ONE DX !!!!
                    pose.pose.orientation.x = 0.6847884219250332
                    pose.pose.orientation.y = -0.018653069577975762
                    pose.pose.orientation.z = 0.7265456014775064     
                    pose.pose.orientation.w = 0.05337011491865343 
                    print("----------------------  ")
                    print("x: {},   y: {},  z: {}".format(round(pose.pose.position.x, 3),round(pose.pose.position.y, 3),round(pose.pose.position.z, 3)))
                    print(" --------------------- ")
                    constraint = Constraints()
                    position_constraints_pose = PositionConstraint()

                    position_constraints_pose.header = pose.header

                    position_constraints_pose.constraint_region.primitives = SolidPrimitive()
                    position_constraints_pose.constraint_region.primitives.type = 1
                    position_constraints_pose.constraint_region.primitives.dimensions = [0.1, 0.1, 0.1]
                    position_constraints_pose.constraint_region.primitive_poses = Pose()
                    position_constraints_pose.constraint_region.primitive_poses = pose
                    constraint.position_constraints = [position_constraints_pose]
                    pilz_pose.goal_constraints = constraint

                    target = self.move_group.set_pose_target(pose)
                    #TARGET is always equal to None, don't know why
                    trajectory = self.move_group.plan(target)
                    if trajectory[0] == False:
                        print("False")
                        self.go_home()
                        failure_cases += 1
                        break
                    #joint_values = self.move_group.get_current_joint_values()
                    time_for_trajectory = float(str(trajectory[1].joint_trajectory.points[-1].time_from_start.secs) + "." +str(trajectory[1].joint_trajectory.points[-1].time_from_start.nsecs))
                    self.move_group.go(target, wait=False)
                    self.data_saver(time_for_trajectory)
                    

                total_pushes += 1
                
                self.go_via_point()
                
            self.go_push()

    def data_saver(self, time_for_trajectory):
        rate                = rospy.Rate(30)
        self.robot_states   = []
        self.camera_finger  = []
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

    def read_robot_data(self, robot_joint_data):#, fing_cam):
            if self.i != self.prev_i:
                self.prev_i = self.i
                ee_state = self.move_group.get_current_pose().pose
                self.robot_states.append([robot_joint_data, ee_state])
                # self.camera_finger.append(fing_cam)

    def format_data_for_saving(self):
        self.robot_states_formated = []
        
        for data_sample_index in range(len(self.robot_states)):
            robot_joint_data = self.robot_states[data_sample_index][0]
            ee_state = self.robot_states[data_sample_index][1]
            self.robot_states_formated.append(list(robot_joint_data.position) + list(robot_joint_data.velocity) + list(robot_joint_data.effort) + 
                                            [ee_state.position.x, ee_state.position.y, ee_state.position.z,
                                            ee_state.orientation.x, ee_state.orientation.y, ee_state.orientation.z, ee_state.orientation.w])# + flattened_jac)
        print(self.robot_states_formated)

    def save_data(self):
        print("robot_states", np.array(self.robot_states).shape)
        # print("camera_finger", np.array(self.camera_finger).shape)


        #create new folder for this experiment:
        folder = str(self.datasave_folder + '/data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        mydir = os.mkdir(folder)

        self.format_data_for_saving()
        T0 = pd.DataFrame(self.robot_states_formated)
        print(T0)
        robot_states_col = ["position_panda_joint1", "position_panda_joint2", "position_panda_joint3", "position_panda_joint4", "position_panda_joint5", "position_panda_joint6", "position_panda_joint7", "position_panda_finger_joint1", "position_panda_finger_joint2",
        "velocity_panda_joint1", "velocity_panda_joint2", "velocity_panda_joint3", "velocity_panda_joint4", "velocity_panda_joint5", "velocity_panda_joint6", "velocity_panda_joint7", "velocity_panda_finger_joint1", "velocity_panda_finger_joint2",
        "effort_panda_joint1", "panda_joint2", "effort_panda_joint3", "effort_panda_joint4", "panda_joint5", "effort_panda_joint6", "effort_panda_joint7", "effort_panda_finger_joint1", "effort_panda_finger_joint2",
        "ee_state_position_x", "ee_state_position_y", "ee_state_position_z", "ee_state_orientation_x", "ee_state_orientation_y", "ee_state_orientation_z", "ee_state_orientation_w"]#"j_11", "j_12", "j_13", "j_14", "j_15", 
        # "j_16", "j_17", "j_21", "j_22", "j_23", "j_24", "j_25", "j_26", "j_27", "j_31", "j_32", "j_33", "j_34", "j_35", "j_36", "j_37", "j_41", "j_42", "j_43", "j_44", "j_45", "j_46", "j_47", "j_51", "j_52", "j_53", "j_54", 
        # "j_55", "j_56", "j_57", "j_61", "j_62", "j_63", "j_64", "j_65", "j_66", "j_67"]

        T0.to_csv(folder + '/robot_state.csv', header=robot_states_col, index=False)
        # np.save(folder + '/camera_finger.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.camera_finger]))


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

    def reset_objects(self):
            # move home first (to get the robot up high enough)
            self.move_group.set_named_target('ready')
            self.move_group.go()

            # Move above the pushback point:
            self.move_group.set_planner_id("PTP")                      # Set to be the straight line planner
            self.move_group.go([0.07130824142401027, 0.8614598760406538, -0.16447389670298204, -0.9640181670858131, 0.13709807091064682, 1.8604888006846219, 0.7095731452172556], wait=True)

            # Move down to the pushback point:
            self.move_group.go([0.06827783184260813, 1.0580782766974726, -0.16370692051658037, -0.9703260839895288, 0.13953893815809099, 2.066282942907594, 0.7095669528161733], wait=True)

            # Push the block back towards the robot to reset the objects.
            self.move_group.set_planner_id("LIN")                      # Set to be the straight line planner
            self.move_group.go([0.13260246841531056, 0.34646877631008416, -0.14746936818756937, -2.2979664011990875, 0.11272908575888033, 2.6471211875279743, 0.6693391410410404], wait=True)

            # move back home (to get the robot up high enough)
            self.move_group.set_planner_id("PTP")                      # Set to be the straight line planner
            self.move_group.go([0.12347067714783183, 0.0752346964597091, -0.17116823404775366, -1.9258203851829596, 0.0030568404994491065, 2.040577341397677, 0.7074800998503142], wait=True)

            # moveto the side to push the block back:
            self.move_group.go([0.5625205920607905, -0.12322370656475262, 0.1751817361488022, -2.1664951839073514, 0.07415023610326978, 2.071069740253273, 1.4965139810825394], wait=True)
            self.move_group.go([0.618022809069619, 0.41956730252173147, 0.1317212792416644, -2.189165379708273, -0.142555658538146, 2.645985071788768, 1.2043076761464278], wait=True)

            # # Push the block away from the robot again to ensure it's out of the way.
            self.move_group.set_planner_id("LIN")                      # Set to be the straight line planner
            self.move_group.go([0.08850941608377269, 0.6825085006845076, 0.01852105101081915, -1.6987854360623158, -0.025516628348523373, 2.4043064999103683, 0.9282770566848588], wait=True)


if __name__ == '__main__':
    robot = FrankaRobot()
    robot.pushing_actions()