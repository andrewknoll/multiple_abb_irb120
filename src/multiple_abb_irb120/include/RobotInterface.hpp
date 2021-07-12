#pragma once

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <string>

class RobotInterface{
    ros::NodeHandle* node_handle;
    const std::string ROBOT_NAMESPACE;
    const std::string PLANNING_GROUP;

    std::shared_ptr <moveit::planning_interface::MoveGroupInterface> move_group;
    const robot_state::JointModelGroup* joint_model_group;

public:
    RobotInterface(std::string planning_group, std::string ns = "/");
    void executeTrajectory(moveit_msgs::RobotTrajectory t);
    
    std::shared_ptr <moveit::planning_interface::MoveGroupInterface> getMoveGroup();
};