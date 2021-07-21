#pragma once

#include <Eigen/Core>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

double normalize(ignition::math::Vector3d point);

ignition::math::Vector3d toIgnitionVector3d(const Eigen::Vector3d& v);
Eigen::Vector3d toEigenVector3d(const ignition::math::Vector3d& v);
std::map<std::string, double> readParameters(std::string filename);