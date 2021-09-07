#pragma once

#include <Eigen/Core>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>

namespace utils{
    geometry_msgs::Pose getAdjustedSpherePose(geometry_msgs::Pose sphereWorldPose, geometry_msgs::Point base_position, tf2::Quaternion orientation);
    double calculateInitialComponent(int index, double offset, double size, int resolution);
    geometry_msgs::Pose calculateInitialPos(int indices[3], double offset[3], double size[3], int resolution[3]);
    bool isNear(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, double distance);
    ignition::math::Pose3d toIgnitionPose3d(const geometry_msgs::Pose& p);
    ignition::math::Vector3d toIgnitionVector3d(const Eigen::Vector3d& v);
    Eigen::Vector3d toEigenVector3d(const ignition::math::Vector3d& v);
    std::map<std::string, double> readParameters(std::string filename);
}