#include "utils.hpp"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Pose.h>

bool isNear(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, double distance) {
  double x = pose1.position.x - pose2.position.x;
  double y = pose1.position.y - pose2.position.y;
  double z = pose1.position.z - pose2.position.z;
  return (sqrt(x*x + y*y + z*z) < distance);
}

ignition::math::Pose3d toIgnitionPose3d(const geometry_msgs::Pose& p){
  ignition::math::Quaternion<double> qua(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  ignition::math::Pose3d pose(p.position.x, p.position.y, p.position.z, qua.Roll(), qua.Pitch(), qua.Yaw());
  return pose;
}

ignition::math::Vector3d toIgnitionVector3d(const Eigen::Vector3d& v){
    return ignition::math::Vector3d(v(0), v(1), v(2));
}

Eigen::Vector3d toEigenVector3d(const ignition::math::Vector3d& v){
    return Eigen::Vector3d(v.X(), v.Y(), v.Z());
}

std::map<std::string, double> readParameters(std::string filename) {
  std::map<std::string, double> parameters;

  //Load parameters from config file
  std::ifstream paramFile(filename);
  if (!paramFile.is_open())
  {
    std::cerr << "Unable to open file[" << filename << "]" << std::endl;
    return parameters;
  }
  std::bitset<10> flags;

  std::string key, value;
  while (!paramFile.eof() && flags.to_ulong() < 0x3FF) {
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
  }
  return parameters;
}