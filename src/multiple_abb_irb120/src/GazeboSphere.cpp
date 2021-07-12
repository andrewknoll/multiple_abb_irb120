#include <ros/ros.h>
#include <ros/service_client.h>
#include "GazeboSphere.hpp"

GazeboSphere::GazeboSphere(ros::NodeHandle* nh) : nh(nh) {
    ros::ServiceClient gazebo_spawn_client = nh->serviceClient<gazebo_msgs::SpawnModel>(GAZEBO_SPAWN_SDF_SERVICE);
    position_client = nh->serviceClient<gazebo_msgs::SetModelState>(GAZEBO_SET_MODEL_STATE);

    gazebo_msgs::SpawnModel model;
    model.request.model_xml = fromFileToString(ros::package::getPath("multiple_abb_irb120") + SPHERE_SDF);
    model.request.model_name = "sphere";
    model.request.reference_frame="world";
    
    gazebo_spawn_client.call(model);
}

void GazeboSphere::setPosition(int x, int y, int z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

void GazeboSphere::update() {
    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = "sphere";
    modelstate.pose.position.x = x;
    modelstate.pose.position.y = y;
    modelstate.pose.position.z = z;
    modelstate.pose.orientation.x = 0;
    modelstate.pose.orientation.y = 0;
    modelstate.pose.orientation.z = 0;
    modelstate.pose.orientation.w = 0;

    position_client.call(modelstate);
}