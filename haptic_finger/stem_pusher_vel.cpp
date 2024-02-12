#include <cmath>
#include <iostream>
#include <fstream>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h> 
#include <Eigen/Dense>


float stem_pose = 0.0;
void wrist_rot_cb(const std_msgs::Float64MultiArray& rotation) 
{
  stem_pose = rotation.data[0];
}

int main(int argc, char** argv) {
  try {
    ros::init(argc, argv, "robot_pose_pub");
    ros::NodeHandle n;
    ros::Publisher robot_pose_pub = n.advertise<std_msgs::Float64MultiArray>("robot_pose", 1000);
    ros::Publisher robot_vel_pub = n.advertise<std_msgs::Float64MultiArray>("robot_vel", 1000);
    ros::Subscriber wrist_rot = n.subscribe("/stem_pose", 1000, wrist_rot_cb);

    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // std::array<double, 7> q_goal = {{0.14360167152468625, 0.1753777056788718, -0.2196985128072391, -1.365035858023108, -0.15087520535954108, 3.1017061897913636, -2.018819763140546}};
    std::array<double, 7> q_goal = {{-0.00635813,0.0372822,-0.0373876,-1.30535,0.0145356,2.85452,0.787199}}; // strawberry stem
    // std::array<double, 7> q_goal = {{0.000189747,-0.348658,-0.0224457,-1.67147,-0.0154906,2.95908,0.788193}}; // behind stem to avoid contact
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);

    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    
    std::string filename = "/home/alessandro/tactile_control_ale/src/haptic_finger_control/RT-data/robot_cart_data/robot_cart_vel.csv";
    std::vector< std::vector<double> > robot_traj;
    
    double time = 0.0;
    double T = 2;
    // double a = 0.8 / pow(T, 2); // linear motion
    double a = 0.6 / pow(T, 2); // arc motion
    double v_y = 0;
    double w_z = 0;
    double w_end_of_motion = 0;
    double w_x = 0;
    double v_z = 0;
    // double a_z = 0.2 / pow(T, 2); // this is for proactive controller
    double a_z = 1.8 * 0.15 / pow(T, 2); // this is for reactive controller
    double a_w = 2*M_PI / (1.5*pow(T, 2));
    
    double stem_t = 0.0;
    double stem_t1 = 0.0;
    double stem_t2 = 0.0;
    double e = 0;
    double e_dot = 0;
    double w_z_des = 0;

    // P gain for reactive :
    // double k_p = 8.0; // 10.0 for linear motion and 8.0 for circular trajectory
    // P gain for proactive :
    double k_p = 10.0;
    double k_d = k_p / 8.0;

    
    robot.control([=, &time](const franka::RobotState& robot_state,
                                         franka::Duration period) mutable -> franka::CartesianVelocities {
      time += period.toSec();
      std::vector<double> EE_pose_vec = {robot_state.O_T_EE[0], robot_state.O_T_EE[1], robot_state.O_T_EE[2], robot_state.O_T_EE[3], robot_state.O_T_EE[4], robot_state.O_T_EE[5], robot_state.O_T_EE[6],
                                   robot_state.O_T_EE[7], robot_state.O_T_EE[8], robot_state.O_T_EE[9], robot_state.O_T_EE[10], robot_state.O_T_EE[11], robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                                  robot_state.O_T_EE[14], robot_state.O_T_EE[15]};
      std_msgs::Float64MultiArray robot_pose_msg;
      robot_pose_msg.data.clear();
      robot_pose_msg.data.resize(17);
      EE_pose_vec.push_back(0.0); //this is the flag to continue/end subscribing fo python script subscriber
      robot_pose_msg.data = EE_pose_vec;
      robot_pose_pub.publish(robot_pose_msg);

      // linear motion bang-bang reference trajectory
      if (time < T/2){
        v_y = - a * time;
        w_x = - a_w * time;
        v_z = a_z * time;
      } else if (time < T){
        v_y = a * time - a * T;
        w_x = a_w * time - a_w * T;
        v_z = -a_z * time + a_z * T;
      } else if (time < 1.2 * T){ // wait for a bit at the end of motion and then return
        v_y = 0.0;
        w_x = 0.0;
        v_z = 0;
        if (time > (1.000 * T) and (time < 1.0006 * T)){
          w_end_of_motion = w_z;
        }
        w_z = (-w_end_of_motion) * (time- T) + w_end_of_motion;
      } else if (time < 1.7 * T){ // here the task is finished but we move the robot to the initial pose
        v_y = a * (time - 1.2 * T);
        w_x = a_w * (time - 1.2 * T);
        v_z = -a_z * (time - 1.2 * T);
        w_z = (-w_end_of_motion) * (time- T) + w_end_of_motion;
      } else if (time < 2.2 * T){
        v_y = - a * time + 2.2 * a * T;
        w_x = - a_w * time + 2.2 * a_w * T;
        v_z = a_z * time - 2.2 * a_z * T;
        w_z = (-w_end_of_motion) * (time- T) + w_end_of_motion;
      }

      if (time < T){ // do it only for the forward path
        // Reactive system: rotational motion PD controller
        // if (stem_pose < 0.63){
        // if (time > 0.78){
        //   stem_t = stem_pose;
        //   if (stem_t != stem_t1){
        //     // e = -(stem_t - 0.45); // negative because rotation around z is positive in opposite
        //     // e_dot = stem_t - stem_t1;
        //     e = -(stem_t - stem_t1);
        //     e_dot = -(stem_t - stem_t2);
        //     if (abs(e) > 0.1){
        //       e = e * 0.5;
        //     }
        //     if (stem_t1 == 0.0 && stem_t2 == 0.0){
        //       e = 0.001;
        //       e_dot = 0.001;
        //     } else if (stem_t1 != 0.0 && stem_t2 == 0.0){
        //       std::cout << "if 2 ..." << std::endl;
        //       e = 0.001;
        //       e_dot = 0.001;
        //     }
        //     w_z_des = w_z_des + 0.08*(k_p * e + k_d * e_dot);
        //     // w_z_des = w_z_des + 0.06*(k_p * e);
        //   }  
        //   w_z = w_z + 0.0625 * (w_z_des - w_z);

        //   stem_t2 = stem_t1;
        //   stem_t1 = stem_t;
        // }

        // Proactive system: rotational motion PD controller
        // if ((stem_pose < 0.460 || stem_pose > 0.469) && stem_pose != 0.0){
        if (time > 0.78){
          stem_t = stem_pose;
          if (stem_t != stem_t1){
            e = -(stem_t - stem_t1)*4.0; // negative because rotation around z is positive in opposite
            e_dot = -(stem_t - stem_t2)*4.0;  //derivative <--- approximated 
            if (abs(e) > 0.1){
              e = e * 0.2;
            }
            if (stem_t1 == 0.0 && stem_t2 == 0.0){      //no clue of what is this
              e = 0.001;
              e_dot = 0.0;
            } else if (stem_t1 != 0.0 && stem_t2 == 0.0){
              std::cout << "if 2 ..." << std::endl;  //print to the console
              e = 0.001;
              e_dot = 0.001;
            }
            w_z_des = w_z_des + 0.08*(k_p * e + k_d * e_dot);     //control action
            // w_z_des = w_z_des + 0.06*(k_p * e);
          }  
          w_z = w_z + 0.0625 * (w_z_des - w_z);

          stem_t2 = stem_t1;
          stem_t1 = stem_t;
        }

        // if (int(round(time*1000)) % 5 == 0){
        //   // std::cout << "w_z_des: " << w_z_des << std::endl;
        //   std::cout << "time    : " << time << ", w_z : " << w_z_des << std::endl;
        // }

        // log traj data
        std::vector<double> robot_vel_vec = {0, v_y, v_z, w_x, 0.0, 0};
        robot_traj.push_back(robot_vel_vec);
        std_msgs::Float64MultiArray robot_vel_msg;
        robot_vel_msg.data = robot_vel_vec;
        robot_vel_pub.publish(robot_vel_msg);
      }

      ros::spinOnce();

      franka::CartesianVelocities output = {{0, v_y, v_z, w_x, 0, w_z}};

      if (time > T and time < 2.2*T) {
        std_msgs::Float64MultiArray robot_pose_msg;
        robot_pose_msg.data.clear();
        robot_pose_msg.data.resize(17);
        int n = EE_pose_vec.size();
        EE_pose_vec[n-1] = 1.0;
        robot_pose_msg.data = EE_pose_vec ;
        robot_pose_pub.publish(robot_pose_msg);
        
      } else if (time > 2.2 * T){
        
        std::ofstream RobotPoseCSV;   
        RobotPoseCSV.open(filename);
        RobotPoseCSV << "v_y" << "," << "w_z" << "," << "w_z_des" << std::endl;
        for (int i = 0; i < robot_traj.size(); i++)
        {
          RobotPoseCSV << robot_traj[i][0] << "," << robot_traj[i][1] << "," << robot_traj[i][2]<< std::endl;
        }
        return franka::MotionFinished(output);
      }
      return output;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;   //print to console
    return -1; 
  }
  return 0;
}