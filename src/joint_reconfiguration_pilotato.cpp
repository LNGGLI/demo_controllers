// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <demo_controllers/joint_reconfiguration_pilotato.h>

#include <cmath>

#include <realtime_tools/realtime_buffer.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace demo_controllers {


bool JointReconfPilotato::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointReconfPilotato: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointReconfPilotato: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointReconfPilotato: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointReconfPilotato: Exception getting joint handles: " << e.what());
      return false;
    }
  }


  // inizializzazione del buffer realtime e del subscriber

  commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
  sub_command_ = node_handle.subscribe<trajectory_msgs::JointTrajectoryPoint>("/joint_commands", 1, &JointReconfPilotato::commandCB, this);
  pub_command_ = node_handle.advertise<trajectory_msgs::JointTrajectoryPoint>("/joint_commands_letti", 1);
  return true;
}

void JointReconfPilotato::starting(const ros::Time& /* time */) {

  std::vector<double> current_positions(n_joints_, 0.0);

    for (std::size_t i = 0; i < n_joints_; ++i)
    {
      current_positions[i] = position_joint_handles_[i].getPosition();
    }
    commands_buffer_.initRT(current_positions);

  elapsed_time_ = ros::Duration(0.0);
}


void JointReconfPilotato::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {

  //trajectory_msgs::JointTrajectoryPoint msg;                                    
  std::vector<double> & commands = *commands_buffer_.readFromRT();
  
  // std::cout << ros::Time::now().toSec() - tempo_prec_ << "\n";
  // tempo_prec_ = ros::Time::now().toSec();
  // for(int i=0; i< 7; i++){
  //   msg.positions.push_back(commands[i]);
  // }
  // pub_command_.publish(msg);


  for(int i = 0;i<7;i++){
    position_joint_handles_[i].setCommand(commands[i]);
  }

    
}
  
  void JointReconfPilotato::commandCB(const trajectory_msgs::JointTrajectoryPointConstPtr& msg){
    if(msg->positions.size()!=7)
    {
      ROS_ERROR_STREAM("Dimension of command (" << msg->positions.size() << ") does not match number of joints. Not executing!");
      return;
    }
    commands_buffer_.writeFromNonRT(msg->positions);
  }


}  // namespace demo_controllers

PLUGINLIB_EXPORT_CLASS(demo_controllers::JointReconfPilotato,
                       controller_interface::ControllerBase)

// PLUGINLIB_EXPORT_CLASS(name_of_your_controller_package::NameOfYourControllerClass,
//                      controller_interface::ControllerBase)
