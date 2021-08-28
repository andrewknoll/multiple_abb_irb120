#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "Grid.hpp"
#include "MassSpringDamping.hpp"
#include <chrono>
#include <multiple_abb_irb120/GrabPetition.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>
#include <tuple>

namespace gazebo
{
  class DeformableObject : public ModelPlugin
  {
  protected:
    class SphereGrasp {
    private:
      physics::LinkPtr link;
      geometry_msgs::Pose pose_cache;
      const double RADIUS;
      const double EPSILON = 0.075;
      std::string link_name, robot_name;
      //ros::Subscriber sub;
      physics::WorldPtr world;
      bool up = true;
      std::shared_ptr<std::thread> updater;

      //void setGrabbedSpherePosCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
      void updatePose();
    public:
      //SphereGrasp(physics::LinkPtr link, double radius, ros::Subscriber sub);
      //SphereGrasp(physics::LinkPtr link, double radius, std::shared_ptr<ros::NodeHandle> n, std::string end_effector_name);
      SphereGrasp(physics::LinkPtr link, double radius, physics::WorldPtr world, std::string robot_name, std::string end_effector_name);
      void shutdown();
      ~SphereGrasp();
    };
  private:

    //http://gazebosim.org/tutorials?tut=guided_i6
    //A node use for ROS transport
    std::shared_ptr<ros::NodeHandle> rosNode;

    //ROS subscribers
    ros::Subscriber grabSub;

    // Pointer to the model
    physics::ModelPtr model;

    //Pointer to the world
    physics::WorldPtr world;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    std::map<std::tuple<int, int>, std::shared_ptr<SphereGrasp> > grasps;

    double offset_x = 0.0, offset_y = 0.0, offset_z = 1.0, width = 5.0, height = 5.0, radius = 0.025;
    double mass = 1.0, stiffness = 1.0, damping = 0.1;
    int vertical_res = 10, horizontal_res = 10;
    bool grid_initialized = false;
    bool gravity = true;

    bool testing = false;

    using GridPtr = std::shared_ptr<multiple_abb_irb120::Grid>;
    using MSDPtr = std::shared_ptr<MassSpringDamping>;

    GridPtr grid;
    MSDPtr msd;

    std::chrono::steady_clock::time_point t0, t;

    void grabCallback(const multiple_abb_irb120::GrabPetition::ConstPtr& msg);
    void getRobotNameAndLink(std::string full_name, std::string& robot_name, std::string& link_name, std::string delimiter);

  public:
    DeformableObject();
    ~DeformableObject();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate();
  };
    GZ_REGISTER_MODEL_PLUGIN(DeformableObject)
}