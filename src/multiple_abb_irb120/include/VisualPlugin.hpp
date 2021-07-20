#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <chrono>

namespace gazebo
{
  class DeformableVisual : public ModelPlugin
  {
    // Pointer to the model
  private:
    // Pointer to the model
    physics::VisualPtr visual;

  public:
    DeformableVisual();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  };
    GZ_REGISTER_VISUAL_PLUGIN(DeformableVisual)
}