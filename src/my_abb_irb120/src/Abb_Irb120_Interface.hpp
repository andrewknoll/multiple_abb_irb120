#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>


class Abb_Irb120_Interface{
    ros::Publisher commander;

public:
    AbbIrb120_Interface(std::string node, unsigned int size = 1000);
    void sendTrajectory(trajectory_msgs::JointTrajectory<std::vector> t);
};