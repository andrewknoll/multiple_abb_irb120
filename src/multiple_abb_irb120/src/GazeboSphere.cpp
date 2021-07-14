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

geometry_msgs::Point pointConstructor(float x, float y, float z) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std::atomic<unsigned int> GazeboSphere::currentID(0);

GazeboSphere::GazeboSphere(ros::NodeHandle* nh, float x, float y, float z) : nh(nh), ID(++currentID), initPos(pointConstructor(x, y, z)) {
    
    ros::ServiceClient gazebo_spawn_client = nh->serviceClient<gazebo_msgs::SpawnModel>(GAZEBO_SPAWN_SDF_SERVICE);
    position_client = nh->serviceClient<gazebo_msgs::SetModelState>(GAZEBO_SET_MODEL_STATE);

    gazebo_msgs::SpawnModel model;
    model.request.model_xml = fromFileToString(ros::package::getPath("multiple_abb_irb120") + SPHERE_SDF);
    model.request.model_name = "sphere" + std::to_string(ID);
    model.request.reference_frame="world";
    setPosition(x, y, z);
    model.request.initial_pose.position = initPos;
    
    gazebo_spawn_client.call(model);
}

void GazeboSphere::setPosition(float x, float y, float z) {
    this->pos.x = x;
    this->pos.y = y;
    this->pos.z = z > 0? z : 0;
}

void GazeboSphere::setF(float x, float y, float z) {
    this->force[0] = x;
    this->force[1] = y;
    this->force[2] = z;
}

void GazeboSphere::setAcc(float x, float y, float z) {
    this->acc[0] = x;
    this->acc[1] = y;
    this->acc[2] = z;
}

void GazeboSphere::setVel(float x, float y, float z) {
    this->vel[0] = x;
    this->vel[1] = y;
    this->vel[2] = z;
}

void GazeboSphere::setPosition(geometry_msgs::Point p){
    this->pos = p;
    if(this->pos.z < 0){
        this->pos.z = 0;
    }
}

void GazeboSphere::setF(Eigen::Vector3f v) {
    this->force = v;
}

void GazeboSphere::setAcc(Eigen::Vector3f v) {
    this->acc = v;
}

void GazeboSphere::setVel(Eigen::Vector3f v) {
    this->vel = v;
}

geometry_msgs::Point GazeboSphere::getInitialPos(){
    return initPos;
}

geometry_msgs::Point GazeboSphere::getCurrentPos(){
    return pos;
}

Eigen::Vector3f GazeboSphere::getF(){
    return force;
}

Eigen::Vector3f GazeboSphere::getAcc(){
    return acc;
}

Eigen::Vector3f GazeboSphere::getVel(){
    return vel;
}

void GazeboSphere::update() {
    gazebo_msgs::SetModelState model_state_petition;
    gazebo_msgs::ModelState& model_state = model_state_petition.request.model_state;
    model_state.model_name = "sphere" + std::to_string(ID);
    model_state.pose.position = pos;
    model_state.pose.orientation.x = 0;
    model_state.pose.orientation.y = 0;
    model_state.pose.orientation.z = 0;
    model_state.pose.orientation.w = 0;

    position_client.call(model_state_petition);
}