#include <ros/service_client.h>
#include <ros/ros.h>
#include "GazeboSphere.hpp"
#include "gazebo/common/common.hh"
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include "gazebo/gazebo.hh"
#include <ros/package.h>
#include <string>
#include <iostream>
#include <atomic>

std::string fromFileToString(std::string filename) {
  std::ifstream f(filename);
  std::stringstream buffer;
  buffer << f.rdbuf();

  return buffer.str();
}

std::atomic<unsigned int> GazeboSphere::currentID(0);

GazeboSphere::GazeboSphere(ros::NodeHandle* nh, float x, float y, float z) : nh(nh), ID(++currentID) {
    ros::ServiceClient gazebo_spawn_client = nh->serviceClient<gazebo_msgs::SpawnModel>(GAZEBO_SPAWN_SDF_SERVICE);
    position_client = nh->serviceClient<gazebo_msgs::SetModelState>(GAZEBO_SET_MODEL_STATE);

    gazebo_msgs::SpawnModel model;
    model.request.model_xml = fromFileToString(ros::package::getPath("multiple_abb_irb120_testing") + SPHERE_SDF);
    model.request.model_name = "sphere" + std::to_string(ID);
    model.request.reference_frame="world";
    setPosition(x, y, z);
    model.request.initial_pose.position.x = x;
    model.request.initial_pose.position.y = y;
    model.request.initial_pose.position.z = z;
    
    gazebo_spawn_client.call(model);
}

void GazeboSphere::setPosition(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

void GazeboSphere::update() {
    gazebo_msgs::SetModelState model_state_petition;
    gazebo_msgs::ModelState& model_state = model_state_petition.request.model_state;
    model_state.model_name = "sphere" + std::to_string(ID);
    model_state.pose.position.x = x;
    model_state.pose.position.y = y;
    model_state.pose.position.z = z;
    model_state.pose.orientation.x = 0;
    model_state.pose.orientation.y = 0;
    model_state.pose.orientation.z = 0;
    model_state.pose.orientation.w = 0;

    position_client.call(model_state_petition);
}
