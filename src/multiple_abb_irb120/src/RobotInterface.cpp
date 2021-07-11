/*
    std::unique_ptr<ros::NodeHandle> nodeHandler;
    const std::string ROBOT_NAMESPACE;
    const std::string PLANNING_GROUP;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    const robot_state::JointModelGroup* joint_model_group;
*/
#include "RobotInterface.hpp"
#include <ros/console.h>

RobotInterface::RobotInterface(std::string planning_group, std::string ns) :
    ROBOT_NAMESPACE(ns),
    PLANNING_GROUP(planning_group) 
{
    std::cout << "Creating " << ns << std::endl;
    std::string temp = (ns == "/" ? ns : ns + "_") + planning_group;
    node_handle = new ros::NodeHandle("/" + ns);
    moveit::planning_interface::MoveGroupInterface::Options opt(temp, "/" + ns + "/robot_description", *node_handle);
    move_group = std::make_shared <moveit::planning_interface::MoveGroupInterface> (opt);
    //joint_model_group = move_group->getCurrentState()->getJointModelGroup(temp);

    ROS_INFO_NAMED("robot_interface", "Started robot in namespace: %s", ns.c_str());
}

void RobotInterface::executeTrajectory(moveit_msgs::RobotTrajectory t) {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    my_plan.trajectory_ = t;
    move_group->execute(my_plan);
}

std::shared_ptr <moveit::planning_interface::MoveGroupInterface>  RobotInterface::getMoveGroup(){
    return move_group;
}
