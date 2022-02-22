// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <demo_controllers/joint_reconfiguration_pilotato.h>
#include <demo_controllers/check_realtime.h>
#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace demo_controllers {

ros::CallbackQueue my_queue_;

bool JointReconfPilotato::init(hardware_interface::RobotHW *robot_hardware,
                               ros::NodeHandle &node_handle) {
  position_joint_interface_ =
      robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR("JointReconfPilotato: Error getting position joint interface "
              "from hardware!");
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
      position_joint_handles_[i] =
          position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException &e) {
      ROS_ERROR_STREAM(
          "JointReconfPilotato: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  // inizializzazione del buffer realtime e del subscriber
  commands_buffer_.resize(7);
  current_positions_.resize(7);

  /* PUBLISHER E SUBSCRIBER node_handle */
  
  // sub_command_ = node_handle.subscribe<trajectory_msgs::JointTrajectoryPoint>(
  //     "/joint_commands", 1, &JointReconfPilotato::commandCB, this);
  nh_ = ros::NodeHandle(node_handle);

  pub_command_update_ =
      node_handle.advertise<trajectory_msgs::JointTrajectoryPoint>(
          "/joint_commands_lettiUpdate", 1);
 
  node_handle.setCallbackQueue(&my_queue_);

  // nh_ = ros::NodeHandle(node_handle);
   
  pub_command_cb_ = node_handle.advertise<trajectory_msgs::JointTrajectoryPoint>(
      "/joint_commands_lettiCB", 1);

  sub_command_ = node_handle.subscribe<trajectory_msgs::JointTrajectoryPoint>(
      "/joint_commands", 1, &JointReconfPilotato::commandCB,this);


  return true;
}


void JointReconfPilotato::starting(const ros::Time & /* time */) {


  driver_running_ = true;
  
  for (std::size_t i = 0; i < n_joints_; ++i) {
    current_positions_[i] = position_joint_handles_[i].getPosition();
  }
  

  elapsed_time_ = ros::Duration(0.0);
  


  // Entra nell'update
  
  spinner_thread_ = std::thread([this]() {
     
    if (!check_realtime())
      throw std::runtime_error("REALTIME NOT AVAILABLE");

    if (!set_realtime_SCHED_FIFO())
      throw std::runtime_error("ERROR IN set_realtime_SCHED_FIFO");

    ros::WallDuration dur(0.1);
    while(ros::ok() && driver_running_ ){ 
      std::cout << "Chiamo callAvailable \n"; 
      my_queue_.callAvailable(dur);
    }

  });
 


}

void JointReconfPilotato::update(const ros::Time & /*time*/,
                                 const ros::Duration &period) {

  // trajectory_msgs::JointTrajectoryPoint msg;

  
  // std::cout << " \n Sono nell'update \n";

  //  for (int i = 0; i < 7; i++) {
  //   position_joint_handles_[i].setCommand(current_positions_[i]);
  // }
  // msg.positions = commands_buffer_;
  // pub_command_update_.publish(msg);

  // for (int i = 0; i < 7; i++) {
  //   position_joint_handles_[i].setCommand(current_positions_[i]);
  // }
}

void JointReconfPilotato::stopping(const ros::Time&){

  std::cout << "Fermato il driver e effettuato Join del thread";
  driver_running_ = false;
  spinner_thread_.join();


}


void JointReconfPilotato::commandCB(
    const trajectory_msgs::JointTrajectoryPointConstPtr &msg) {

  
   commands_buffer_ = msg->positions;
  
  for (int i = 0; i < 7; i++) {
    position_joint_handles_[i].setCommand(commands_buffer_[i]);
  }
  std::cout << " CB eseguita \n";
  pub_command_cb_.publish(msg);

}



} // namespace demo_controllers

PLUGINLIB_EXPORT_CLASS(demo_controllers::JointReconfPilotato,
                       controller_interface::ControllerBase)

// PLUGINLIB_EXPORT_CLASS(name_of_your_controller_package::NameOfYourControllerClass,
//                      controller_interface::ControllerBase)
