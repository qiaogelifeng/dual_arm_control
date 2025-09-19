#ifndef PINOCCHIO_ROS_CONTROLLER_H
#define PINOCCHIO_ROS_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pinocchio/fwd.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include <string>
#include <memory>

namespace joint_controller {

#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 30 

class JointController {
public:
  /**
   * @brief Constructor
   * @param nh ROS node handle
   */
  JointController(ros::NodeHandle& nh);
  
  /**
   * @brief Destructor
   */
  ~JointController() = default;
  
  /**
   * @brief Initialize the controller
   * @return True if initialization is successful
   */
  bool init();
  
  /**
   * @brief Run the controller main loop
   */
  void run();

private:
  /**
   * @brief Callback for joint state messages
   * @param msg Joint state message
   */
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  
  /**
   * @brief Callback for trajectory commands
   * @param msg Joint trajectory message
   */
  void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
  
  /**
   * @brief Load robot model from URDF
   * @param urdf_path Path to URDF file
   * @return True if successful
   */
  bool loadRobotModel(const std::string& urdf_path);
  
  /**
   * @brief Compute control commands using Pinocchio
   */
  void computeControl();
  
  /**
   * @brief Publish control commands
   */
  void publishCommands();

  // ROS node handle
  ros::NodeHandle nh_;
  
  // // ROS publishers and subscribers
  ros::Subscriber joint_state_sub_;
  ros::Subscriber trajectory_sub_;
  ros::Publisher joint_command_pub_;
  ros::Publisher controller_state_pub_;
  
  // // TF broadcaster for end effector pose
  // tf2_ros::TransformBroadcaster tf_broadcaster_;
  
  // // Pinocchio model and data
  // pinocchio::ModelTpl<double> model_;
  // pinocchio::Data data_;
  
  // // Joint names and indices
  std::vector<std::string> joint_names_;
  std::vector<size_t> joint_indices_;
  
  // // Current joint states
  Eigen::VectorXd q_;  // Joint positions
  Eigen::VectorXd dq_; // Joint velocities
  
  // // Desired joint states
  Eigen::VectorXd q_des_;  // Desired joint positions
  Eigen::VectorXd dq_des_; // Desired joint velocities
  Eigen::VectorXd ddq_des_;// Desired joint accelerations
  
  // // Control gains
  Eigen::VectorXd kp_; // Proportional gains
  Eigen::VectorXd kd_; // Derivative gains
  
  // // Control commands
  Eigen::VectorXd tau_; // Joint torques
  
  // Controller parameters
  double control_rate_;
  std::string urdf_path_;
  std::string end_effector_frame_;
  
  // Flag to indicate if joint states have been received
  bool has_joint_states_;
};

} // namespace pinocchio_ros_controller

#endif // PINOCCHIO_ROS_CONTROLLER_H
