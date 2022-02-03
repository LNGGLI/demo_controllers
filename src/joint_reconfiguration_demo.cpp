// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <demo_controllers/joint_reconfiguration_demo.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace demo_controllers {

std::array<double, 7> q_goal{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
double tf = 10; // s

bool JointReconfigControllerDemo::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointReconfigControllerDemo: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointReconfigControllerDemo: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointReconfigControllerDemo: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointReconfigControllerDemo: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  
  

  return true;
}

void JointReconfigControllerDemo::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    q_init_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
}

void JointReconfigControllerDemo::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  
  std::array<double,7> q_command;
  
  // time = il tempo attuale
  // period = tempo passato dall'ultima chiamata ad update ( 1 ms )
  
  elapsed_time_ += period;
  double tau = elapsed_time_.toSec()/tf;

  // costruzione polinomio quintico q(t) = q_init + (q_goal - q_init)*q(t/tf)
  for(int i = 0; i< 7; i++){
    q_command[i] = q_init_[i] + (q_goal[i] - q_init_[i])*(6*pow(tau,5)-15*pow(tau,4)+10*pow(tau,3));
  }

  if(elapsed_time_.toSec() < tf) {
    for(int i = 0; i< 7; i++){
      position_joint_handles_[i].setCommand(q_command[i]);
    }
  }
  else{
    for(int i = 0; i< 7; i++){
      position_joint_handles_[i].setCommand(q_goal[i]);
    }
  }

  
 

  
}

}  // namespace demo_controllers

PLUGINLIB_EXPORT_CLASS(demo_controllers::JointReconfigControllerDemo,
                       controller_interface::ControllerBase)

// PLUGINLIB_EXPORT_CLASS(name_of_your_controller_package::NameOfYourControllerClass,
//                      controller_interface::ControllerBase)
