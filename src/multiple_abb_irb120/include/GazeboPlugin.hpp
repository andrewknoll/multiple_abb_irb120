#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "Grid.hpp"
#include "MassSpringDamping.hpp"
#include <chrono>
#include <multiple_abb_irb120/GrabPetition.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>

#include <mutex>
#include <condition_variable>

namespace gazebo
{
  class DeformableObject : public ModelPlugin
  {
  protected:
    class SphereGrasp {
    private:
      std::mutex mtx;
      std::condition_variable cv;
      volatile bool must_update = false;
      bool up = true;
      std::shared_ptr<std::thread> updater;
      physics::LinkPtr link;
      geometry_msgs::Pose pose_cache;
      ros::Subscriber sub;
      void setGrabbedSpherePosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
      void updatePose();
    public:
      SphereGrasp(physics::LinkPtr link, ros::Subscriber sub);
      SphereGrasp(physics::LinkPtr link, std::shared_ptr<ros::NodeHandle> n, std::string topic);
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

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    std::map<std::string, std::shared_ptr<SphereGrasp> > subscribers;

    double offset_x = 0.0, offset_y = 0.0, offset_z = 1.0, width = 5.0, height = 5.0;
    double mass = 1.0, stiffness = 1.0, damping = 0.1;
    int vertical_res = 10, horizontal_res = 10;
    bool grid_initialized = false;

    using GridPtr = std::shared_ptr<multiple_abb_irb120::Grid>;
    using MSDPtr = std::shared_ptr<MassSpringDamping>;

    GridPtr grid;
    MSDPtr msd;

    std::chrono::steady_clock::time_point t0, t;

    void grabCallback(const multiple_abb_irb120::GrabPetition::ConstPtr& msg);

  public:
    DeformableObject();
    ~DeformableObject();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate();
  };
    GZ_REGISTER_MODEL_PLUGIN(DeformableObject)
}