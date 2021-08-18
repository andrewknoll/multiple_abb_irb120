#pragma once

#include <Eigen/Core>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Pose.h>

bool isNear(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, double distance);
ignition::math::Pose3d toIgnitionPose3d(const geometry_msgs::Pose& p);
ignition::math::Vector3d toIgnitionVector3d(const Eigen::Vector3d& v);
Eigen::Vector3d toEigenVector3d(const ignition::math::Vector3d& v);
std::map<std::string, double> readParameters(std::string filename);