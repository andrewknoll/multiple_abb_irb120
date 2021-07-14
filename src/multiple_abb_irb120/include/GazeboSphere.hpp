#pragma once

#include <ros/service_client.h>
#include <ros/ros.h>
#include <atomic>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>

const std::string GAZEBO_SPAWN_SDF_SERVICE = "/gazebo/spawn_sdf_model";
const std::string GAZEBO_SET_MODEL_STATE = "/gazebo/set_model_state";
const std::string SPHERE_SDF = "/urdf/sphere.sdf";

std::string fromFileToString(std::string filename);
geometry_msgs::Point pointConstructor(float x, float y, float z);

class GazeboSphere {
private:
    static std::atomic<unsigned int> currentID;
    ros::NodeHandle* nh;
    const geometry_msgs::Point initPos;
    geometry_msgs::Point pos;
    Eigen::Vector3f force = Eigen::Vector3f::Zero(), acc = Eigen::Vector3f::Zero(), vel = Eigen::Vector3f::Zero();
    ros::ServiceClient position_client;
public:
    const unsigned int ID;

    GazeboSphere(ros::NodeHandle* nh, float x, float y, float z);

    geometry_msgs::Point getInitialPos();
    geometry_msgs::Point getCurrentPos();
    Eigen::Vector3f getF();
    Eigen::Vector3f getAcc();
    Eigen::Vector3f getVel();

    void setPosition(float x, float y, float z);
    void setF(float x, float y, float z);
    void setAcc(float x, float y, float z);
    void setVel(float x, float y, float z);
    void setPosition(geometry_msgs::Point p);
    void setF(Eigen::Vector3f v);
    void setAcc(Eigen::Vector3f v);
    void setVel(Eigen::Vector3f v);

    void update();
};