#include <ros/ros.h>
#include "joint_controller/joint_controller.h"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

using namespace joint_controller;

int main(int argc, char**argv) {
  using namespace pinocchio;
  const std::string urdf_file = std::string("/home/zm/DualArm_ws/src/dual_arm_control/dual_arm_description/urdf/openarm/openarm_bimanual.urdf");
  Model model;
  pinocchio::urdf::buildModel(urdf_file, model);
  std::cout << "Model name : "<< model.name<<std::endl;
  Data data(model);
  // print joint informations
  std::cout << "\n=== Joints ===" << std::endl;
  for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i) {
    std::cout << "Joint [" << i << "]: " << model.names[i] 
              << " (Type: " << model.joints[i].shortname() << ")" << std::endl;
  }
  
  // Sample a random configuration
  Eigen::VectorXd q = randomConfiguration(model);
  std::cout << "q: " << q.transpose() << std::endl;
  // Perform the forward kinematics over the kinematic tree
  forwardKinematics(model, data, q);  
   // Print out the placement of each joint of the kinematic tree
  for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left << model.names[joint_id] << ": " << std::fixed
              << std::setprecision(2) << data.oMi[joint_id].translation().transpose() << std::endl; 
  return 0;
}  