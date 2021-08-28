#include "RobotInterface.hpp"
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <string>
#include <thread>
#include <ros/console.h>
#include <geometry_msgs/Point.h>

#include <execinfo.h>
       #include <stdio.h>
       #include <stdlib.h>
       #include <unistd.h>



RobotInterface::RobotInterface(geometry_msgs::Point position, std::string planning_group, std::string ns) :
    BASE_POSITION(position),
    ROBOT_NAMESPACE(ns),
    PLANNING_GROUP(planning_group) 
{
    std::cout << "Creating " << ns << std::endl;
    std::string temp = (ns == "/" ? ns : ns + "_") + planning_group;
    node_handle = std::make_shared<ros::NodeHandle>("/" + ns);
    moveit::planning_interface::MoveGroupInterface::Options opt(temp, "/" + ns + "/robot_description", *node_handle);
    move_group = std::make_shared <moveit::planning_interface::MoveGroupInterface> (opt);
    //joint_model_group = move_group->getCurrentState()->getJointModelGroup(temp);
    //publisherThread = std::make_shared<std::thread>(&RobotInterface::publishEndEffectorPose, this);

    ROS_INFO_NAMED("robot_interface", "Started robot in namespace: %s", ns.c_str());
}

void RobotInterface::shutdown(){
    //up = false;
    //publisherThread->join();
    //delete node_handle;
}

RobotInterface::~RobotInterface(){
    ROS_INFO_NAMED("robot_interface", "Shutting down robot in namespace: %s", ROBOT_NAMESPACE.c_str());
    //shutdown();
}

void RobotInterface::publishEndEffectorPose(){
    try{
        endEffPublish  = std::make_shared<ros::Publisher>(node_handle->advertise<geometry_msgs::PoseStamped>("end_effector_pose", 1000));
    }
    catch(std::exception& e){
        ROS_ERROR_NAMED("robot_interface", "Failed to create publisher for end effector pose: %s", e.what());
        return;
    }
    while(up && ros::ok()) {
        try{
            auto s = move_group->getCurrentPose();
            endEffPublish->publish(s);
        }
        catch(...){
            std::cout << "ERROR" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if(!ros::ok()){
        std::cout << "ROS IS SAD" << std::endl;
    }
    if(!up){
        std::cout << "UP NOT :(" << std::endl;
    }
}

void RobotInterface::executeTrajectory(moveit_msgs::RobotTrajectory t) {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    my_plan.trajectory_ = t;
    move_group->execute(my_plan);
}

std::shared_ptr <moveit::planning_interface::MoveGroupInterface>  RobotInterface::getMoveGroup() const {
    return move_group;
}

geometry_msgs::Point RobotInterface::getBasePosition() const {
    return BASE_POSITION;
}