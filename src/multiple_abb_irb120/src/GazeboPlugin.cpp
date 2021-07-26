#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "GazeboPlugin.hpp"
#include "Grid.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <ros/package.h>
#include "MassSpringDamping.hpp"
#include "utils.hpp"

using namespace gazebo;


DeformableObject::DeformableObject() : ModelPlugin()
{
  std::cout << "Starting deformable object plugin..." << std::endl;
  t0 = std::chrono::steady_clock::now();

}

void DeformableObject::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model = _parent;

  const std::string PACKAGE_PATH = ros::package::getPath("multiple_abb_irb120");
  std::map<std::string, double> parameters = readParameters(PACKAGE_PATH + "/config/grid.config");
  if(parameters.count("width") != 0) width = parameters["width"];
  if(parameters.count("height") != 0) height = parameters["height"];
  if(parameters.count("vertical_res") != 0) vertical_res = (int)parameters["vertical_res"];
  if(parameters.count("horizontal_res") != 0) horizontal_res = (int)parameters["horizontal_res"];
  if(parameters.count("offset_x") != 0) offset_x = parameters["offset_x"];
  if(parameters.count("offset_y") != 0) offset_y = parameters["offset_y"];
  if(parameters.count("offset_z") != 0) offset_z = parameters["offset_z"];
  if(parameters.count("mass") != 0) mass = parameters["mass"];
  if(parameters.count("stiffness") != 0) stiffness = parameters["stiffness"];
  if(parameters.count("damping") != 0) damping = parameters["damping"];

  std::cout << "Initializing Mass Spring Damping system..." << std::endl;
  std::cout << "Parameters: " << std::endl;
  std::cout << "width: " << width << std::endl;
  std::cout << "height: " << height << std::endl;
  std::cout << "vertical_res: " << vertical_res << std::endl;
  std::cout << "horizontal_res: " << horizontal_res << std::endl;
  std::cout << "offset_x: " << offset_x << std::endl;
  std::cout << "offset_y: " << offset_y << std::endl;
  std::cout << "offset_z: " << offset_z << std::endl;
  std::cout << "mass: " << mass << std::endl;
  std::cout << "stiffness: " << stiffness << std::endl;
  std::cout << "damping: " << damping << std::endl;
  msd = std::make_shared<MassSpringDamping>(mass, stiffness, damping, false);

  if(!grid_initialized){
    std::cout << "Initializing grid..." << std::endl;
    grid = std::make_shared<multiple_abb_irb120::Grid>(model, offset_x, offset_y, offset_z, width, height, vertical_res, horizontal_res);
    std::cout << "Grid: Success!" << std::endl;
  }

  // Listen to the update event. This event is broadcast every
      // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&DeformableObject::OnUpdate, this));

  std::cout << "Deformable Object Plugin Initialized." << std::endl;
}

void DeformableObject::OnUpdate() {
  t = std::chrono::steady_clock::now();
  msd->computePositions(grid/*, std::chrono::duration_cast<std::chrono::microseconds>(t - t0).count() * 1e-06*/);
  grid->update();
  t0 = t;
}