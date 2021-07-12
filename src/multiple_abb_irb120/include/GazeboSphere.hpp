#include <ros/ros.h>
#include <ros/service_client.h>

const std::string GAZEBO_SPAWN_SDF_SERVICE = "/gazebo/spawn_sdf_model";
const std::string GAZEBO_SET_MODEL_STATE = "/gazebo/set_model_state";
const std::string SPHERE_SDF = "/urdf/sphere.sdf";

class GazeboSphere {
private:
    ros::NodeHandle* nh;
    int x, y, z;
    ros::ServiceClient position_client;
public:
    GazeboSphere(ros::NodeHandle* nh);
    void setPosition(int x, int y, int z);
    void update();
    
}