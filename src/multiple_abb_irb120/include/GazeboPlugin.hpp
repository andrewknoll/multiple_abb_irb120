#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "Grid.hpp"
#include "MassSpringDamping.hpp"
#include <chrono>

namespace gazebo
{
  class DeformableObject : public ModelPlugin
  {
    // Pointer to the model
  private:
    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    double offset_x = 0.0, offset_y = 0.0, offset_z = 1.0, width = 5.0, height = 5.0;
    double mass = 1.0, stiffness = 1.0, damping = 0.1;
    int vertical_res = 10, horizontal_res = 10;
    bool grid_initialized = false;

    using GridPtr = std::shared_ptr<multiple_abb_irb120::Grid>;
    using MSDPtr = std::shared_ptr<MassSpringDamping>;

    GridPtr grid;
    MSDPtr msd;

    std::chrono::steady_clock::time_point t0, t;

  public:
    DeformableObject();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate();
  };
    GZ_REGISTER_MODEL_PLUGIN(DeformableObject)
}