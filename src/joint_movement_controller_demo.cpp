// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <demo_controllers/joint_movement_controller_demo.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace demo_controllers {

bool JointMovementControllerDemo::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointMovementControllerDemo: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointMovementControllerDemo: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointMovementControllerDemo: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointMovementControllerDemo: Exception getting joint handles: " << e.what());
      return false;
    }
  }


  return true;
}

void JointMovementControllerDemo::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
}

void JointMovementControllerDemo::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {

  // time = il tempo attuale
  // period = tempo passato dall'ultima chiamata ad update ( 1 ms )

  elapsed_time_ += period;

  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.01;
  for (size_t i = 0; i < 7; ++i){
    if (i == 4) {
      position_joint_handles_[i].setCommand(initial_pose_[i] - delta_angle);
    } else {
      position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle);
    }
  }
}

}  // namespace demo

PLUGINLIB_EXPORT_CLASS(demo_controllers::JointMovementControllerDemo,
                       controller_interface::ControllerBase)

// PLUGINLIB_EXPORT_CLASS(name_of_your_controller_package::NameOfYourControllerClass,
//                      controller_interface::ControllerBase)
