// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool CartesianImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  // publisher for trying
  pub_robot_state_     = node_handle.advertise<std_msgs::Float64MultiArray>("panda_robot_state_topic",1);;

  // change the topic name to /target_pose if you want to use the topic coming from interactive_marker.py
  sub_equilibrium_pose_ = node_handle.subscribe(
      "/target_pose", 20, &CartesianImpedanceExampleController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  
  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceExampleController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return true;
}

void CartesianImpedanceExampleController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state     --------> hwith tis commented the robot go quickly to (0,0,0)
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());


  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void CartesianImpedanceExampleController::update(const ros::Time& time,
                                                 const ros::Duration& period) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // autodefined vector for testing
  //Eigen::Vector3d position_d_data_collection_ (0.7, -0.3,  0.85);
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_target_;
 
  Eigen::Quaterniond orientation_d_(      // check this shit
    -0.0166441, // w component
     0.726858,   // x component
     0.0265717,  // y component
     0.686072   // z component
  );

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }

  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);



  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task +  coriolis + tau_nullspace; 
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // Impedence force computation
  Eigen::Matrix<double, 6, 1> force_mat;
  force_mat = -cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq);

  // Publish data collection message

  std_msgs::Float64MultiArray robot_state_msg;
  std_msgs::Float64MultiArray pendu_state_msg;

  Eigen::Map<Eigen::Matrix<double,7,1>> q_d(robot_state.q_d.data());
  Eigen::Map<Eigen::Matrix<double,7,1>> dq_d(robot_state.dq_d.data());
  Eigen::Map<Eigen::Matrix<double,7,1>> ddq_d(robot_state.ddq_d.data());

  Eigen::Map<Eigen::Matrix<double,7,1>> tau_J(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double,4,4>> O_T_EE(robot_state.O_T_EE.data());
  Eigen::Map<Eigen::Matrix<double,4,4>> O_T_EE_d(robot_state.O_T_EE_d.data());
  Eigen::Map<Eigen::Matrix<double,4,4>> O_T_EE_c(robot_state.O_T_EE_c.data());
  Eigen::Map<Eigen::Matrix<double,4,4>> EE_T_K(robot_state.EE_T_K.data());
  Eigen::Map<Eigen::Matrix<double,6,1>> O_F_ext_hat_K(robot_state.O_F_ext_hat_K.data());
  Eigen::Map<Eigen::Matrix<double,6,1>> K_F_ext_hat_K(robot_state.K_F_ext_hat_K.data());
  double m_ee = robot_state.m_ee;
  double m_load = robot_state.m_load;
  double m_total = robot_state.m_total;
  Eigen::Matrix<double, 6, 1> d_error = jacobian * dq;

  robot_state_msg.data = {q[0],  q[1],  q[2],  q[3],  q[4],  q[5],  q[6],
                          q_d[0],q_d[1],q_d[2],q_d[3],q_d[4],q_d[5],q_d[6],
                          dq[0], dq[1], dq[2], dq[3], dq[4], dq[5], dq[6],
                          dq_d[0], dq_d[1], dq_d[2], dq_d[3], dq_d[4], dq_d[5], dq_d[6],
                          ddq_d[0],ddq_d[1],ddq_d[2],ddq_d[3],ddq_d[4],ddq_d[5],ddq_d[6],

                          tau_J[0],tau_J[1],tau_J[2],tau_J[3],tau_J[4],tau_J[5],tau_J[6],
                          tau_J_d[0],tau_J_d[1],tau_J_d[2],tau_J_d[3],tau_J_d[4],tau_J_d[5],tau_J_d[6],

                          O_T_EE(0,0),O_T_EE(1,0),O_T_EE(2,0),O_T_EE(3,0),
                          O_T_EE(0,1),O_T_EE(1,1),O_T_EE(2,1),O_T_EE(3,1),
                          O_T_EE(0,2),O_T_EE(1,2),O_T_EE(2,2),O_T_EE(3,2),
                          O_T_EE(0,3),O_T_EE(1,3),O_T_EE(2,3),O_T_EE(3,3),


                          O_T_EE_d(0,0),O_T_EE_d(1,0),O_T_EE_d(2,0),O_T_EE_d(3,0),
                          O_T_EE_d(0,1),O_T_EE_d(1,1),O_T_EE_d(2,1),O_T_EE_d(3,1),
                          O_T_EE_d(0,2),O_T_EE_d(1,2),O_T_EE_d(2,2),O_T_EE_d(3,2),
                          O_T_EE_d(0,3),O_T_EE_d(1,3),O_T_EE_d(2,3),O_T_EE_d(3,3),

                          O_T_EE_c(0,0),O_T_EE_c(1,0),O_T_EE_c(2,0),O_T_EE_c(3,0),
                          O_T_EE_c(0,1),O_T_EE_c(1,1),O_T_EE_c(2,1),O_T_EE_c(3,1),
                          O_T_EE_c(0,2),O_T_EE_c(1,2),O_T_EE_c(2,2),O_T_EE_c(3,2),
                          O_T_EE_c(0,3),O_T_EE_c(1,3),O_T_EE_c(2,3),O_T_EE_c(3,3),

                          EE_T_K(0,0),EE_T_K(1,0),EE_T_K(2,0),EE_T_K(3,0),
                          EE_T_K(0,1),EE_T_K(1,1),EE_T_K(2,1),EE_T_K(3,1),
                          EE_T_K(0,2),EE_T_K(1,2),EE_T_K(2,2),EE_T_K(3,2),
                          EE_T_K(0,3),EE_T_K(1,3),EE_T_K(2,3),EE_T_K(3,3),


                          O_F_ext_hat_K[0],O_F_ext_hat_K[1],O_F_ext_hat_K[2],
                          O_F_ext_hat_K[3],O_F_ext_hat_K[4],O_F_ext_hat_K[5],

                          K_F_ext_hat_K[0],K_F_ext_hat_K[1],K_F_ext_hat_K[2],
                          K_F_ext_hat_K[3],K_F_ext_hat_K[4],K_F_ext_hat_K[5],

                          m_ee,m_load,m_total,
                          error[0],error[1],error[2],error[3],error[4],error[5],
                          d_error[0],d_error[1],d_error[2],d_error[3],d_error[4],d_error[5],

                          force_mat[0],force_mat[1],force_mat[2],force_mat[3],force_mat[4],force_mat[5],
                          jacobian_array[0],jacobian_array[1],jacobian_array[2],
                          jacobian_array[3],jacobian_array[4],jacobian_array[5],
                          jacobian_array[6],jacobian_array[7],jacobian_array[8],
                          jacobian_array[9],jacobian_array[10],jacobian_array[11],
                          jacobian_array[12],jacobian_array[13],jacobian_array[14],
                          jacobian_array[15],jacobian_array[16],jacobian_array[17],
                          jacobian_array[18],jacobian_array[19],jacobian_array[20],
                          jacobian_array[21],jacobian_array[22],jacobian_array[23],
                          jacobian_array[24],jacobian_array[25],jacobian_array[26],
                          jacobian_array[27],jacobian_array[28],jacobian_array[29],
                          jacobian_array[30],jacobian_array[31],jacobian_array[32],
                          jacobian_array[33],jacobian_array[34],jacobian_array[35],
                          jacobian_array[36],jacobian_array[37],jacobian_array[38],
                          jacobian_array[39],jacobian_array[40],jacobian_array[41],

                          coriolis_array[0], coriolis_array[1],coriolis_array[2],
                          coriolis_array[3],coriolis_array[4],coriolis_array[5],
                          coriolis_array[6],
                          period.toSec()//, start_data_collection
                        };
  pub_robot_state_.publish(robot_state_msg);










  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering $ 
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;

  // still not clear the purpose of these lines
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

}

Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceExampleController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void CartesianImpedanceExampleController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
//  subscirbe the orientation
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerBase)
