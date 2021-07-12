#pragma once

#include <ros/service_client.h>
#include <ros/ros.h>
#include <atomic>

const std::string GAZEBO_SPAWN_SDF_SERVICE = "/gazebo/spawn_sdf_model";
const std::string GAZEBO_SET_MODEL_STATE = "/gazebo/set_model_state";
const std::string SPHERE_SDF = "/urdf/sphere.sdf";

std::string fromFileToString(std::string filename);

class GazeboSphere {
private:
    static std::atomic<unsigned int> currentID;
    ros::NodeHandle* nh;
    int x, y, z;
    ros::ServiceClient position_client;
public:
    const unsigned int ID;

    GazeboSphere(ros::NodeHandle* nh);
    void setPosition(int x, int y, int z);
    void update();
};