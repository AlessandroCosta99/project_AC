// franka::Robot (Current RobotState) https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a7ea7074a07b63fcf6933e97b078c7168
// franka::RobotState (current x, y and z) https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html#af372a0081d72bc7b4fe873f99c7b2d8c
// franka::Model (Jacobian) https://frankaemika.github.io/libfranka/classfranka_1_1Model.html
// print_joint_poses.cpp (Cartesian Pose) https://frankaemika.github.io/libfranka/print_joint_poses_8cpp-example.html
// generate_cartesian_pose_motion.cpp (motion) https://frankaemika.github.io/libfranka/generate_cartesian_pose_motion_8cpp-example.html
// Ros subscriber: https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
// franka::CartesianVelocities https://frankaemika.github.io/libfranka/classfranka_1_1CartesianVelocities.html
// cartesian_impedance_control.cpp (ZeroJacobian) https://frankaemika.github.io/libfranka/cartesian_impedance_control_8cpp-example.html#a11

#include <array>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/exception.h>
#include <geometry_msgs/Pose.h>
#include "examples_common.h"

struct DesiredState {
    std::array<double, 3> position;
};

DesiredState desired_state;


void goalPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    desired_state.position = {msg->position.x, msg->position.y, msg->position.z};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pd_controller_node");
    ros::NodeHandle nh;

    ros::Subscriber goal_pose_sub = nh.subscribe("/next_pose", 1000, goalPoseCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::array<double, 3> previous_error = {0.0, 0.0, 0.0};
    ros::Time last_time = ros::Time::now();

    // saving the data
    // std::string filename = "/home/alessandro/data_pd_controller/controller_data.csv";

    try {
        const std::string robot_ip = "172.16.0.2";// Robot ip to be changed

        franka::Robot robot(robot_ip);

        franka::Model model = robot.loadModel();

        setDefaultBehavior(robot);
        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This will move the robot! "
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;
        std::cout << "WARNING: This will run the pd controller"
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
        
        // PD gains
        double Kp = 10; // Set your proportional gain
        double Kd = 0; // Set your derivative gain

        robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
            // Calculate the Jacobian
            auto jacobian = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
            // Read current pose
            std::array<double, 16> pose = robot_state.O_T_EE;
            

            // Calculate position error
            std::array<double, 3> error = {
                pose[12] - desired_state.position[0] , 
                pose[13] - desired_state.position[1] ,
                pose[14] - desired_state.position[2]
            };

            // Calculate the time step
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();
            last_time = current_time;
 
            // error dot
            std::array<double, 3> error_dot = {
                (error[0] - previous_error[0]) / dt,
                (error[1] - previous_error[1]) / dt,
                (error[2] - previous_error[2]) / dt
            };

            std::cout << "error dot " << error_dot[0] << std::endl;
            // Update previous error
            previous_error = error;

            //prints
            std::cout << "current_pose x: " << pose[12] << std::endl;
            std::cout << "desired_pose x: " << desired_state.position[0] << std::endl;
            std::cout << "error position x: " << error[0] <<"<-----" << std::endl;
            std::cout << "current_pose y: " << pose[13] << std::endl;
            std::cout << "desired_pose y: " << desired_state.position[1] << std::endl;
            std::cout << "error position y: " << error[1] << "<-----" << std::endl;
            std::cout << "current_pose z: " << pose[14] << std::endl;
            std::cout << "desired_pose z: " << desired_state.position[2] << std::endl;
            std::cout << "error position z: " << error[2] << "<-----" << std::endl;

 
            // Cartesian PD controller
            std::array<double, 6> F = { // linear velocity
                Kp * error[0] + Kd * error_dot[0],
                Kp * error[1] + Kd * error_dot[1],
                Kp * error[2] + Kd * error_dot[2],
                0, 0, 0 // angular velocity
            };

            std::cout << "F " << F[0] << std::endl;
            std::cout << "F " << F[1] << std::endl;
            std::cout << "F " << F[2] << std::endl;
            std::cout << "F " << F[3] << std::endl;
            std::cout << "F " << F[4] << std::endl;
            std::cout << "F " << F[5] << std::endl;
   
            std::array<double, 7> tau;
            for (int i = 0; i < 7; ++i) {
                tau[i] = 0.0;
                for (int j = 0; j < 6; ++j) {
                    tau[i] += jacobian[i + 7 * j] * F[j];
                }
            std::cout << "tau " << i << ":" << tau[i] << std::endl;
            }

            // saving data in csv
            // std::ofstream DebugCSV;   
            // DebugCSV.open(filename);
            // DebugCSV << "error x" << "," << "Force x" << "," << "tau 0" << "," << "tau 1" <<  "," <<"tau 2" <<"," << "tau 3" << "," <<"tau 4" << std::endl;
            // DebugCSV << error[0] << "," << F[0] << "," << tau[0] << "," << tau[1] << "," << tau[2] << "," << tau[3] << "," << tau[4] << std::endl;
            tau[6] = 0;
            return franka::Torques(tau);
        });

    std::cout << "Finished moving to the desired pose." << std::endl;

    } catch (franka::Exception& e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }

    ros::waitForShutdown();

    return 0;
}