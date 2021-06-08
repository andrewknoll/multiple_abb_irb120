#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace my_controller_pkg{

class MyControllerClass: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
private:
  hardware_interface::JointHandle joint_;
  double init_pos_;
  ros::Subscriber sub_command_;

public:
  /**
   * \brief Store position and velocity command in struct to allow easier realtime buffer usage
   */
  struct Commands
  {
    double position_; // Last commanded position
    double velocity_; // Last commanded velocity
    bool has_velocity_; // false if no velocity command has been specified
  };
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_; // pre-allocated memory that is re-used to set the realtime buffer
  virtual bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
  virtual void starting();
  virtual void update(const ros::Time& time, const ros::Duration& period);
  virtual void stopping();
    void setCommandCB(const std_msgs::Float64ConstPtr& msg);
  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param command
   */
  void setCommand(double pos_target);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *        Also supports a target velocity
   *
   * \param pos_target - position setpoint
   * \param vel_target - velocity setpoint
   */
  void setCommand(double pos_target, double vel_target);
};
} 
