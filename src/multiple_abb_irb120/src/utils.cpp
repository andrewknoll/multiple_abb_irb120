#include "utils.hpp"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>

geometry_msgs::Pose utils::getAdjustedSpherePose(geometry_msgs::Pose sphereWorldPose, geometry_msgs::Point base_position, tf2::Quaternion orientation){
  sphereWorldPose.position.x -= base_position.x;
  sphereWorldPose.position.y -= base_position.y;
  sphereWorldPose.position.z -= base_position.z;

  sphereWorldPose.orientation.x = orientation.x();
  sphereWorldPose.orientation.y = orientation.y();
  sphereWorldPose.orientation.z = orientation.z();
  sphereWorldPose.orientation.w = orientation.w();

  return sphereWorldPose;
}

double utils::calculateInitialComponent(int index, double offset, double size, int resolution) {
    return offset + (resolution > 1 ? (double)(index - (double)(resolution - 1) / 2.0) * (size / (double)(resolution - 1)) : 0);
}

geometry_msgs::Pose utils::calculateInitialPos(int indices[3], double offset[3], double size[3], int resolution[3]){
  geometry_msgs::Pose pose;

  pose.orientation.w = 1.0;
  pose.position.x = calculateInitialComponent(indices[0], offset[0], size[0], resolution[0]);
  pose.position.y = calculateInitialComponent(indices[1], offset[1], size[1], resolution[1]);
  pose.position.z = calculateInitialComponent(indices[2], offset[2], size[2], resolution[2]);

  return pose;
}

bool utils::isNear(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, double distance) {
  double x = pose1.position.x - pose2.position.x;
  double y = pose1.position.y - pose2.position.y;
  double z = pose1.position.z - pose2.position.z;
  return (sqrt(x*x + y*y + z*z) < distance);
}

ignition::math::Pose3d utils::toIgnitionPose3d(const geometry_msgs::Pose& p){
  ignition::math::Quaternion<double> qua(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  ignition::math::Pose3d pose(p.position.x, p.position.y, p.position.z, qua.Roll(), qua.Pitch(), qua.Yaw());
  return pose;
}

ignition::math::Vector3d utils::toIgnitionVector3d(const Eigen::Vector3d& v){
    return ignition::math::Vector3d(v(0), v(1), v(2));
}

Eigen::Vector3d utils::toEigenVector3d(const ignition::math::Vector3d& v){
    return Eigen::Vector3d(v.X(), v.Y(), v.Z());
}

std::map<std::string, double> utils::readParameters(std::string filename) {
  std::map<std::string, double> parameters;

  //Load parameters from config file
  std::ifstream paramFile(filename);
  if (!paramFile.is_open())
  {
    std::cerr << "Unable to open file[" << filename << "]" << std::endl;
    return parameters;
  }
  std::bitset<11> flags;

  std::string key, value;
  while (!paramFile.eof() && flags.to_ulong() < 0x7FF) {
    std::getline(paramFile, key, ':');
    std::getline(paramFile, value);
    if(key == "width") {
      parameters["width"] = stod(value);
      flags[0] = 1;
    }
    else if(key == "height") {
      parameters["height"] = stod(value);
      flags[1] = 1;
    }
    else if(key == "vertical_resolution"){
      parameters["vertical_res"] = stoi(value);
      flags[2] = 1;
    }
    else if(key == "horizontal_resolution"){
      parameters["horizontal_res"] = stoi(value);
      flags[3] = 1;
    }
    else if(key == "offset_x"){
      parameters["offset_x"] = stod(value);
      flags[4] = 1;
    }
    else if(key == "offset_y"){
      parameters["offset_y"] = stod(value);
      flags[5] = 1;
    }
    else if(key == "offset_z"){
      parameters["offset_z"] = stod(value);
      flags[6] = 1;
    }
    else if(key == "mass"){
      parameters["mass"] = stod(value);
      flags[7] = 1;
    }
    else if(key == "damping"){
      parameters["damping"] = stod(value);
      flags[8] = 1;
    }
    else if(key == "stiffness"){
      parameters["stiffness"] = stod(value);
      flags[9] = 1;
    }
    else if(key == "gravity"){
      parameters["gravity"] = (value == "1" || value == "true");
      flags[10] = 1;
    }
  }
  return parameters;
}