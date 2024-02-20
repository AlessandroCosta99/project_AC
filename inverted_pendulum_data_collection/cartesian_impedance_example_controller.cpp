// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
//---------------------------------------
// #include <std_msgs/Float64MultiArray.h>
//---------------------------------------
#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers 
{

  bool CartesianImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                                ros::NodeHandle& node_handle) 
  {
    std::vector<double> cartesian_stiffness_vector;
    std::vector<double> cartesian_damping_vector;

    sub_equilibrium_pose_ = node_handle.subscribe(
        "equilibrium_pose", 20, &CartesianImpedanceExampleController::equilibriumPoseCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    //--------------------------------------------------------
    // initiate three publishers
    pub_debug_state_     = node_handle.advertise<std_msgs::Float64MultiArray>("debug_state_topic",1);
    pub_robot_state_     = node_handle.advertise<std_msgs::Float64MultiArray>("panda_robot_state_topic",1);
    pub_pendu_state_     = node_handle.advertise<std_msgs::Float64MultiArray>("panda_pendu_state_topic",1);
    //--------------------------------------------------------



    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) 
    {
      ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) 
    {
      ROS_ERROR(
          "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) 
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Error getting model interface from hardware");
      return false;
    }
    try 
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id + "_model"));
    } 
    catch (hardware_interface::HardwareInterfaceException& ex) 
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting model handle from interface: "
          << ex.what());
      return false;
    }

    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) 
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Error getting state interface from hardware");
      return false;
    }
    try 
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    } 
    catch (hardware_interface::HardwareInterfaceException& ex) 
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) 
    {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i) 
    {
      try 
      {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      } 
      catch (const hardware_interface::HardwareInterfaceException& ex) 
      {
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

    //---------------------------------------------------

    // For data collection
    start_data_collection = 0.0;

    // For calculating angles
    theta_zx_R = 0.0;
    theta_zy_R = 0.0;
    d_theta_zx_R = 0.0;
    d_theta_zy_R = 0.0;
    prev_theta_zx_R = 0.0;
    prev_theta_zy_R = 0.0;
    last_callback_time = ros::Time::now();
    offset_theta_zx_R = 0.0;
    offset_theta_zy_R = 0.0;

    // For N sec delay
    start_time = ros::Time::now();
    current_time = ros::Time::now();
    
    // Outmost PID
    outer_int_X_error   = 0.0;
    outer_int_Y_error   = 0.0;
    // kp_slow_x_rqt       = 0.0;
    // kp_slow_y_rqt       = 0.0;
    // ki_slow_x_rqt       = 0.0;
    // ki_slow_y_rqt       = 0.0;
    // kd_slow_x_rqt       = 0.0;
    // kd_slow_y_rqt       = 0.0;
    

    // Middle PID
    inner_int_theta_zx_error = 0.0; 
    inner_int_theta_zy_error = 0.0; 
    // kp_zx_rqt           = 0.0;
    // kp_zy_rqt           = 0.0;
    // ki_zx_rqt           = 0.0;
    // ki_zy_rqt           = 0.0;
    // kd_zx_rqt           = 0.0;
    // kd_zy_rqt           = 0.0;
    

    // Innest PID
    int_Vdx_mins_Vcx    = 0.0;
    int_Vdy_mins_Vcy    = 0.0;
    pre_Vdx_mins_Vcx    = 0.0;
    pre_Vdy_mins_Vcy    = 0.0;
    // kp_velo_x_rqt       = 0.0;
    // kp_velo_y_rqt       = 0.0;
    // ki_velo_x_rqt       = 0.0;
    // ki_velo_y_rqt       = 0.0;
    // kd_velo_x_rqt       = 0.0;
    // kd_velo_y_rqt       = 0.0;

    sub_mocap_pose_up_.subscribe(node_handle,"Robot1/pose",1);  // can I add sth like ros::TransportHints().reliable().tcpNoDelay()
    sub_mocap_pose_down_.subscribe(node_handle,"Robot2/pose",1); // 1 should be tuned 
    int queue_size_sync =1; // to tune?
    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(ApproxSyncPolicy(queue_size_sync), sub_mocap_pose_up_,sub_mocap_pose_down_);
    sync_->registerCallback(boost::bind(&CartesianImpedanceExampleController::mocapPoseCallback_sync,this,_1,_2));
    
    
    //---------------------------------------------------
    return true;
  }

  void CartesianImpedanceExampleController::starting(const ros::Time& /*time*/) 
  {
    // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
    // to initial configuration
    franka::RobotState initial_state = state_handle_->getRobotState();
    // get jacobian
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    // convert to eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    // set equilibrium point to current state
    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
    position_d_target_ = initial_transform.translation();
    orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q_initial;

    //--------------------------------------------------------
    // For data collection
    start_data_collection = 0.0;

    // For N sec delay
    start_time = ros::Time::now();

    // For outmost PID integrator
    outer_int_X_error    = 0.0;
    outer_int_Y_error    = 0.0;

    // Middle PID integrator
    inner_int_theta_zx_error = 0.0; 
    inner_int_theta_zy_error = 0.0; 

    // Innest PID integrator
    int_Vdx_mins_Vcx    = 0.0;
    int_Vdy_mins_Vcy    = 0.0;
    //--------------------------------------------------------
  }

  void CartesianImpedanceExampleController::update(const ros::Time& /*time*/,
                                                  const ros::Duration& period) 
  {
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

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;

    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) 
    {
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
    
    //---------------------------------------------------------------------------------------------------
    

    // ***********Outmost PID***********
    Eigen::Matrix<double, 6, 1> outer_error_d_mins_true;  // d_mins_true means desired minus true data
    outer_error_d_mins_true.head(3) << position_d_ - position;
    Eigen::Matrix<double, 6, 1> outer_error_dot_zero_mins_true =  - jacobian * dq;
    outer_int_X_error = outer_int_X_error + period.toSec() * outer_error_d_mins_true[0];
    outer_int_Y_error = outer_int_Y_error + period.toSec() * outer_error_d_mins_true[1];

    // Outmost PID. Output is the desired stabilisation angle 
    double theta_zx_R_d = kp_slow_x_rqt*outer_error_d_mins_true[0] + ki_slow_x_rqt*outer_int_X_error  + kd_slow_x_rqt*outer_error_dot_zero_mins_true[0];
    double theta_zy_R_d = kp_slow_y_rqt*outer_error_d_mins_true[1] + ki_slow_y_rqt*outer_int_Y_error  + kd_slow_y_rqt*outer_error_dot_zero_mins_true[1];
    // ***********Outmost PID***********




    // ***********Middle PID***********
    double inner_theta_zx_error_t_mins_d = theta_zx_R-theta_zx_R_d; //t_mins_d means true minus desired data
    double inner_theta_zy_error_t_mins_d = theta_zy_R-theta_zy_R_d;

    inner_int_theta_zx_error = inner_int_theta_zx_error + period.toSec() * inner_theta_zx_error_t_mins_d;
    inner_int_theta_zy_error = inner_int_theta_zy_error + period.toSec() * inner_theta_zy_error_t_mins_d;

    double delta_x = kp_zx_rqt*inner_theta_zx_error_t_mins_d + ki_zx_rqt*inner_int_theta_zx_error + kd_zx_rqt*(d_theta_zx_R - 0.0); // desired angular velo = 0.0
    double delta_y = kp_zy_rqt*inner_theta_zy_error_t_mins_d + ki_zy_rqt*inner_int_theta_zy_error + kd_zy_rqt*(d_theta_zy_R - 0.0);
    // ***********Middle PID***********



    // ***********innest PID***********

    // test innest loop//
    // delta_x = 0.13;
    // delta_y = -0.13;
    // test innest loop//


    double Vdx_n = delta_x; 
    double Vdy_n = delta_y; 
    Eigen::Matrix<double, 6, 1> Vc_n_min_1 = jacobian * dq; //measured velocity at tn-1
    double Vdx_mins_Vcx = Vdx_n - Vc_n_min_1[0]; // means target end-effector velocity in x minus measured one
    double Vdy_mins_Vcy = Vdy_n - Vc_n_min_1[1];
    double dot_Vdx_mins_Vcx = (Vdx_mins_Vcx- pre_Vdx_mins_Vcx)/period.toSec();
    double dot_Vdy_mins_Vcy = (Vdy_mins_Vcy - pre_Vdy_mins_Vcy)/period.toSec();
    

    int_Vdx_mins_Vcx = int_Vdx_mins_Vcx + period.toSec() * Vdx_mins_Vcx;
    int_Vdy_mins_Vcy = int_Vdy_mins_Vcy + period.toSec() * Vdy_mins_Vcy;

    double f_x = kp_velo_x_rqt * Vdx_mins_Vcx
                + ki_velo_x_rqt * int_Vdx_mins_Vcx
                + kd_velo_x_rqt * dot_Vdx_mins_Vcx;
    double f_y = kp_velo_y_rqt * Vdy_mins_Vcy
                + ki_velo_y_rqt * int_Vdy_mins_Vcy
                + kd_velo_y_rqt * dot_Vdy_mins_Vcy;

    pre_Vdx_mins_Vcx = Vdx_mins_Vcx;
    pre_Vdy_mins_Vcy = Vdy_mins_Vcy;
    // ***********innest PID***********



    // Safety function. Activate torque update in 5 secs.
    Eigen::Matrix<double, 6, 1> force_mat;
    force_mat = -cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq);
    current_time = ros::Time::now();
    float count_down = abs(current_time.toSec() - start_time.toSec());
    if (count_down > 5.0)
    {
      // set data collection activation flag to 1
      Eigen::Matrix<double, 6, 1> eedot = jacobian * dq;
      if (abs(eedot[0]) > 0.002)
      {
        start_data_collection = 1.0;
      }
      // else
      // {
      //   start_data_collection = 0.0;
      // }
      

      // activate PD control
      force_mat[0] = f_x;  //cancel out --- for testing data logging
      force_mat[1] = f_y; 
    }

    tau_task << jacobian.transpose() * force_mat;
    //---------------------------------------------------------------------------------------------------
    
    
    // Desired torque
    tau_d << tau_task + tau_nullspace + coriolis;
    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i) 
    {
      joint_handles_[i].setCommand(tau_d(i));
    }

    //------------------------------------------
    

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
                            period.toSec(), start_data_collection
                          };
    pendu_state_msg.data = {theta_zx_R, theta_zy_R, theta_zx_R, theta_zy_R, d_theta_zx_R, d_theta_zy_R,
                            0.0,        0.0,        0.0,        0.0}; // The last four are predicted data
    pub_robot_state_.publish(robot_state_msg);
    pub_pendu_state_.publish(pendu_state_msg);  


    // Publish debug message
    std_msgs::Float64MultiArray debug_state_msg;
    debug_state_msg.data = {theta_zx_R*(180.0/M_PI),theta_zy_R*(180.0/M_PI),d_theta_zx_R*(180.0/M_PI),d_theta_zy_R*(180.0/M_PI),
                            delta_x,delta_y,Vc_n_min_1[0],Vc_n_min_1[1],Vdx_mins_Vcx,Vdy_mins_Vcy,dot_Vdx_mins_Vcx,dot_Vdy_mins_Vcy,f_x,f_y,
                            start_data_collection,d_error[0],theta_zx_R_d*(180.0/M_PI),theta_zy_R_d*(180.0/M_PI)};
    pub_debug_state_.publish(debug_state_msg);
    //------------------------------------------

    // update parameters changed online either through dynamic reconfigure or through the interactive
    // target by filtering
    cartesian_stiffness_ =
        filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ =
        filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    nullspace_stiffness_ =
        filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_);
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  }

  Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleController::saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d) 
  {  // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++) 
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] =
          tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
  }

  void CartesianImpedanceExampleController::complianceParamCallback(
      franka_example_controllers::compliance_paramConfig& config,
      uint32_t /*level*/) 
  {
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
    // -----------------------------------------------------
    // Outmost PID
    kp_slow_x_rqt       = config.kp_slow_x_rqt;
    kp_slow_y_rqt       = config.kp_slow_y_rqt;
    ki_slow_x_rqt       = config.ki_slow_x_rqt;
    ki_slow_y_rqt       = config.ki_slow_y_rqt;
    kd_slow_x_rqt       = config.kd_slow_x_rqt;
    kd_slow_y_rqt       = config.kd_slow_y_rqt;
    

    // Middle PID
    kp_zx_rqt           = config.kp_zx_rqt;
    kp_zy_rqt           = config.kp_zy_rqt;
    ki_zx_rqt           = config.ki_zx_rqt;
    ki_zy_rqt           = config.ki_zy_rqt;
    kd_zx_rqt           = config.kd_zx_rqt;
    kd_zy_rqt           = config.kd_zy_rqt;
    

    // Innest PID
    kp_velo_x_rqt       = config.kp_velo_x_rqt;
    kp_velo_y_rqt       = config.kp_velo_y_rqt;
    ki_velo_x_rqt       = config.ki_velo_x_rqt;
    ki_velo_y_rqt       = config.ki_velo_y_rqt;
    kd_velo_x_rqt       = config.kd_velo_x_rqt;
    kd_velo_y_rqt       = config.kd_velo_y_rqt;
    
    // For angle calibration
    offset_theta_zx_R   = config.offset_theta_zx_R;
    offset_theta_zy_R   = config.offset_theta_zy_R;

    // ----------------------------------------------------
  }

  void CartesianImpedanceExampleController::equilibriumPoseCallback(
      const geometry_msgs::PoseStampedConstPtr& msg) 
  {
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_);
    position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) 
    {
      orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }
  }

  void CartesianImpedanceExampleController::mocapPoseCallback_sync(const geometry_msgs::PoseStampedConstPtr& msg1, 
                                  const geometry_msgs::PoseStampedConstPtr& msg2)
  {
    current_pose_up_ = msg1->pose;
    current_pose_down_ = msg2->pose;

    theta_zx_R = std::atan2(current_pose_up_.position.x - current_pose_down_.position.x, -(current_pose_up_.position.y - current_pose_down_.position.y));
    theta_zy_R = std::atan2(current_pose_up_.position.z - current_pose_down_.position.z, -(current_pose_up_.position.y - current_pose_down_.position.y));

    theta_zx_R = theta_zx_R - offset_theta_zx_R*M_PI/180.0;
    theta_zy_R = theta_zy_R - offset_theta_zy_R*M_PI/180.0;
    //calculate the rate of update 
    ros::Time current_callback_time;
    current_callback_time = ros::Time::now();
    double dt_callback = abs(current_callback_time.toSec() - last_callback_time.toSec());
    d_theta_zx_R = (theta_zx_R - prev_theta_zx_R)/dt_callback; // tune 0.00833, which is the freq of this callback
    d_theta_zy_R = (theta_zy_R - prev_theta_zy_R)/dt_callback;

    // record old
    prev_theta_zx_R = theta_zx_R; 
    prev_theta_zy_R = theta_zy_R; 
  }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerBase)
