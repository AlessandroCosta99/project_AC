// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

//-------------------------------------------------
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
//-------------------------------------------------

namespace franka_example_controllers 
{

    class CartesianImpedanceExampleController : public controller_interface::MultiInterfaceController<
                                                    franka_hw::FrankaModelInterface,
                                                    hardware_interface::EffortJointInterface,
                                                    franka_hw::FrankaStateInterface> 
    {
        public:
            bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
            void starting(const ros::Time&) override;
            void update(const ros::Time&, const ros::Duration& period) override;

        private:
            // Saturation
            Eigen::Matrix<double, 7, 1> saturateTorqueRate(
                const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

            std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
            std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
            std::vector<hardware_interface::JointHandle> joint_handles_;

            double filter_params_{0.005};
            double nullspace_stiffness_{20.0};
            double nullspace_stiffness_target_{20.0};
            const double delta_tau_max_{1.0};
            Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
            Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
            Eigen::Matrix<double, 6, 6> cartesian_damping_;
            Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
            Eigen::Matrix<double, 7, 1> q_d_nullspace_;
            Eigen::Vector3d position_d_;
            Eigen::Quaterniond orientation_d_;
            std::mutex position_and_orientation_d_target_mutex_;
            Eigen::Vector3d position_d_target_;
            Eigen::Quaterniond orientation_d_target_;

            // Dynamic reconfigure
            std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
                dynamic_server_compliance_param_;
            ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
            void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                                        uint32_t level);

            // Equilibrium pose subscriber
            ros::Subscriber sub_equilibrium_pose_;
            void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);



            //--------------------------------------

            // For N sec delay safety function
            ros::Time start_time;
            ros::Time current_time;

            // Outmost PID
            double outer_int_X_error; // int(Xd-X) over dt, To do: reset in the safety function
            double outer_int_Y_error;
            double kp_slow_x_rqt;     // PID gains
            double kp_slow_y_rqt;
            double kd_slow_x_rqt;
            double kd_slow_y_rqt;
            double ki_slow_x_rqt;
            double ki_slow_y_rqt;

            // Middle PID
            double inner_int_theta_zx_error; // for integral action 
            double inner_int_theta_zy_error; // 
            double kp_zx_rqt;
            double kp_zy_rqt;
            double kd_zx_rqt;
            double kd_zy_rqt;
            double ki_zx_rqt;
            double ki_zy_rqt;

            // Innest PID
            double int_Vdx_mins_Vcx; // integration used in velo tracking 
            double int_Vdy_mins_Vcy; // d means desired. c means commanded 
            double pre_Vdx_mins_Vcx; //commanded or measured velocity at the previous time step
            double pre_Vdy_mins_Vcy;
            double kp_velo_x_rqt;
            double kp_velo_y_rqt;
            double ki_velo_x_rqt;
            double ki_velo_y_rqt;
            double kd_velo_x_rqt;
            double kd_velo_y_rqt;
            

            // For angles 
            double theta_zx_R;
            double theta_zy_R;
            double d_theta_zx_R;
            double d_theta_zy_R;

            // For publishing data
            ros::Publisher  pub_debug_state_;  // use for debugging
            ros::Publisher  pub_robot_state_;  // publish robot states for data collection
            ros::Publisher  pub_pendu_state_;  // publish pendu angles for data collection
            double start_data_collection;      // flag. activates data collection if this flag is set to 1


            // Used in mocap callback function
            double offset_theta_zx_R; // note!! unit is degree!
            double offset_theta_zy_R;
            double prev_theta_zx_R;
            double prev_theta_zy_R;
            ros::Time last_callback_time;
            geometry_msgs::Pose current_pose_up_;
            geometry_msgs::Pose current_pose_down_;
            message_filters::Subscriber<geometry_msgs::PoseStamped> sub_mocap_pose_up_;
            message_filters::Subscriber<geometry_msgs::PoseStamped> sub_mocap_pose_down_;
            typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,geometry_msgs::PoseStamped> ApproxSyncPolicy;
            std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

            void mocapPoseCallback_sync(const geometry_msgs::PoseStampedConstPtr& msg1, 
                                    const geometry_msgs::PoseStampedConstPtr& msg2);
            //--------------------------------------
    };

}  // namespace franka_example_controllers
