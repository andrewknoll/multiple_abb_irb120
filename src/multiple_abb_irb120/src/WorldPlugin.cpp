//Gazebo v9.0.0
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
//#include "GazeboPlugin.hpp"
#include "ros/ros.h"
#include "ros/package.h"

std::string fromFileToString(std::string filename) {
  std::ifstream f(filename);
  std::stringstream buffer;
  buffer << f.rdbuf();

  return buffer.str();
}

const std::string MODEL_PLUGIN_FILENAME = "libgazebo_plugin.so";
const std::string MODEL_PLUGIN_NAME = "gazebo_plugin";
const std::string PACKAGE_NAME = "multiple_abb_irb120";

namespace gazebo
{
class Factory : public WorldPlugin
{
  physics::WorldPtr world;
  float width = 5.0, height = 5.0;
  int vertical_res = 10, horizontal_res = 10;

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    /*if(_sdf->HasElement("width")) width = _sdf->Get<float>("width");
    if(_sdf->HasElement("height")) height = _sdf->Get<float>("height");
    if(_sdf->HasElement("vertical_res")) vertical_res = _sdf->Get<int>("vertical_res");
    if(_sdf->HasElement("horizontal_res")) horizontal_res = _sdf->Get<int>("horizontal_res");*/
        
    this->world = _parent;

    physics::ModelPtr model = this->world->ModelByName("grid");

gazebo::physics::SphereShapePtr sphere =
      boost::dynamic_pointer_cast<gazebo::physics::SphereShape>(
        model->GetChildLink("sphere1")->GetCollision("collision")->GetShape());

        gazebo::physics::SphereShapePtr sphereCopy = boost::make_shared<physics::SphereShape>(*sphere);

    std::string suffix;

    if(model != nullptr) {
      //Spawn all links with their respective collision spheres
      for(int i = 0; i < vertical_res; i++){
          for(int j = 0; j < horizontal_res; j++){
              printf("Creating link...\n");
              suffix = "_" + std::to_string(i) + "_" + std::to_string(j);
              //Create a new link
              physics::LinkPtr link = this->world->Physics()->CreateLink(model);
              link->SetInitialRelativePose(ignition::math::Pose3d((double)i, (double)j, 1.0, 0.0, 0.0, 0.0));
              link->SetName("link" + suffix);
             // physics::CollisionPtr col = this->world->Physics()->CreateCollision("sphere", link);
              //col->SetShape(sphereCopy);
              //sphereCopy->Init();
              //printf("voy a llorar link...\n");
              //col->Init();
              //printf(" link...\n");
          }
      }
      //model->Update();
    }
    else{
      printf("Error: No 'grid' model found.\n");
    }
    
    printf("Started World Grid Plugin\n");

    /* ModelPluginPtr plugin = DeformableObject::Create(MODEL_PLUGIN_FILENAME, MODEL_PLUGIN_NAME);
    plugin->Load(model, _sdf); */
    //this->world->LoadPlugin("libvisual_plugin.so", "visual_plugin", model);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}