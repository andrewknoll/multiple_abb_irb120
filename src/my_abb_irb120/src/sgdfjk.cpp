#include "controller.hpp"
#include <pluginlib/class_list_macros.h>

namespace my_controller_pkg {

/// Controller initialization in non-realtime
bool MyControllerClass::init(hardware_interface::EffortJointInterface *robot,
                            ros::NodeHandle &n)
{
  std::string joint_name;
  if (!n.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  joint_ = robot->getHandle(joint_name);
//  if (!joint_)
//  {
//    ROS_ERROR("MyController could not find joint named '%s'",
//              joint_name.c_str());
//    return false;
//  }
  // Start command subscriber
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &MyControllerClass::setCommandCB, this);
  return true;
}


/// Controller startup in realtime
void MyControllerClass::starting()
{
  init_pos_ = joint_.getPosition();
}


/// Controller update loop in realtime
void MyControllerClass::update(const ros::Time& time, const ros::Duration& period)
{
  command_struct_ = *(command_.readFromRT());
  double command_position = command_struct_.position_;

  double desired_pos = init_pos_ + 0.5 * sin(ros::Time::now().toSec());
  double current_pos = joint_.getPosition();
    double commanded_effort = -1000 * (current_pos - command_position);
  //joint_.commanded_effort_ = -10 * (current_pos - desired_pos);
  joint_.setCommand(commanded_effort);

}


/// Controller stopping in realtime
void MyControllerClass::stopping()
{}

void MyControllerClass::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  setCommand(msg->data);
}
// Set the joint position command
void MyControllerClass::setCommand(double pos_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false; // Flag to ignore the velocity command since our setCommand method did not include it

  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  command_.writeFromNonRT(command_struct_);
}

// Set the joint position command with a velocity command as well
void MyControllerClass::setCommand(double pos_command, double vel_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.velocity_ = vel_command;
  command_struct_.has_velocity_ = true;

  command_.writeFromNonRT(command_struct_);
}

} // namespace


// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(my_controller_pkg::MyControllerClass, 
                         controller_interface::ControllerBase);