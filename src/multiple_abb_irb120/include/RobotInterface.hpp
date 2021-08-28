#pragma once

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/Point.h>

#define BT_BUF_SIZE 100

class RobotInterface{
    std::shared_ptr<ros::NodeHandle> node_handle;
    std::shared_ptr<std::thread> publisherThread;
    const std::string ROBOT_NAMESPACE;
    const std::string PLANNING_GROUP;
    std::shared_ptr<ros::Publisher> endEffPublish;
    const geometry_msgs::Point BASE_POSITION;
    bool up = true;

    std::shared_ptr <moveit::planning_interface::MoveGroupInterface> move_group;
    const robot_state::JointModelGroup* joint_model_group;

    void publishEndEffectorPose();

    int nptrs;
    void *buffer[BT_BUF_SIZE];
    char **strings;


public:
    RobotInterface(geometry_msgs::Point position, std::string planning_group, std::string ns = "/");
    ~RobotInterface();
    void executeTrajectory(moveit_msgs::RobotTrajectory t);
    void shutdown();

    geometry_msgs::Point getBasePosition() const;
    
    std::shared_ptr <moveit::planning_interface::MoveGroupInterface> getMoveGroup() const;
};