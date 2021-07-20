//Gazebo v9.0.0
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
//#include "GazeboPlugin.hpp"
#include "ros/ros.h"
#include "ros/package.h"
#include "utils.hpp"

const double SPHERE_MASS = 0.1;
const double SPHERE_RADIUS = 0.025;

double calculateInitialComponent(int index, double offset, double size, int resolution) {
    return offset - (size / 2) + (double)index * (size / (double)resolution);
}

gazebo::msgs::Pose* calculateInitialPos(int indices[3], double offset[3], double size[3], int resolution[3]){
    gazebo::msgs::Pose* pos = new gazebo::msgs::Pose();
    gazebo::msgs::Vector3d* position = new gazebo::msgs::Vector3d();
    position->set_x(calculateInitialComponent(indices[0], offset[0], size[0], resolution[0]));
    position->set_y(calculateInitialComponent(indices[1], offset[1], size[1], resolution[1]));
    position->set_z(calculateInitialComponent(indices[2], offset[2], size[2], resolution[2]));
    gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
    pos->set_allocated_position(position);
    pos->set_allocated_orientation(orientation);

    return pos;
}

std::string fromFileToString(std::string filename) {
  std::ifstream f(filename);
  std::stringstream buffer;
  buffer << f.rdbuf();

  return buffer.str();
}

const std::string MODEL_PLUGIN_NAME = "gazebo_plugin";
const std::string PACKAGE_NAME = "multiple_abb_irb120";

namespace gazebo
{
class Factory : public WorldPlugin
{
  physics::WorldPtr world;
  float width = 5.0, height = 5.0;
  int vertical_res = 10, horizontal_res = 10;
  float offset_x = 0.0, offset_y = 0.0, offset_z = 1.0;

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    const std::string PACKAGE_PATH = ros::package::getPath("multiple_abb_irb120");
    std::map<std::string, double> parameters = readParameters(PACKAGE_PATH + "/config/grid.config");
    if(parameters.count("width") != 0) width = parameters["width"];
    if(parameters.count("height") != 0) height = parameters["height"];
    if(parameters.count("vertical_res") != 0) vertical_res = (int)parameters["vertical_res"];
    if(parameters.count("horizontal_res") != 0) horizontal_res = (int)parameters["horizontal_res"];
    if(parameters.count("offset_x") != 0) offset_x = parameters["offset_x"];
    if(parameters.count("offset_y") != 0) offset_y = parameters["offset_y"];
    if(parameters.count("offset_z") != 0) offset_z = parameters["offset_z"];
        
    this->world = _parent;

    std::string suffix;

    sdf::SDFPtr modelSDF(new sdf::SDF);
    sdf::init(modelSDF);

    gazebo::msgs::Model model;
    model.set_name("grid");

    std::cout << "El momento de la verdad xd" << std::endl;

    gazebo::msgs::Plugin* plugin = model.add_plugin();
    plugin->set_name(MODEL_PLUGIN_NAME);
    plugin->set_filename("lib" + MODEL_PLUGIN_NAME + ".so");

    double offset[3] = {offset_x, offset_y, offset_z};
    double size[3] = {width, height, 1};
    int resolution[3] = {vertical_res, horizontal_res, 1};
    int indices[3] = {0, 0, 0};

    //Spawn all links with their respective collision spheres
    for(int i = 0; i < vertical_res; i++){
      indices[0] = i;
      for(int j = 0; j < horizontal_res; j++){
        indices[1] = j;
        printf("Creating link...\n");
        suffix = "_" + std::to_string(i * (int)width + j);

        //Create a new link
        gazebo::msgs::AddSphereLink(model, SPHERE_MASS, SPHERE_RADIUS);
        gazebo::msgs::Link* link = model.mutable_link(model.link_size()-1);

        link->set_name("link" + suffix);
        link->set_allocated_pose(calculateInitialPos(indices, offset, size, resolution));

      }
    }

    modelSDF->Root()->InsertElement(gazebo::msgs::ModelToSDF(model));

    _parent->InsertModelSDF(*modelSDF);

    printf("Started World Grid Plugin\n");
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}