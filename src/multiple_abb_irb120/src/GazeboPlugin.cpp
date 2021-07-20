#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "GazeboPlugin.hpp"
#include "Grid.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <ros/package.h>
#include "MassSpringDamping.hpp"

using namespace gazebo;


DeformableObject::DeformableObject() : ModelPlugin()
{
  std::cout << "Starting deformable object plugin..." << std::endl;
  //Get parameters from grid.config file
  std::string config_file = ros::package::getPath("multiple_abb_irb120") + "/config/grid.config";

  //Load parameters from config file
  std::ifstream paramFile(config_file);
  if (!paramFile.is_open())
  {
    std::cerr << "Unable to open file[" << config_file << "]" << std::endl;
    return;
  }
  std::bitset<10> flags;

  std::string key, value;
  while (!paramFile.eof() && flags.to_ulong() < 10) {
    std::getline(paramFile, key, '=');
    std::getline(paramFile, value);

    if(key == "width") {
      width = stof(value);
      flags[0] = 1;
    }
    else if(key == "height") {
      height = stof(value);
      flags[1] = 1;
    }
    else if(key == "vertical_resolution"){
      vertical_res = stof(value);
      flags[2] = 1;
    }
    else if(key == "horizontal_resolution"){
      horizontal_res = stof(value);
      flags[3] = 1;
    }
    else if(key == "offset_x"){
      offset_x = stof(value);
      flags[4] = 1;
    }
    else if(key == "offset_y"){
      offset_y = stof(value);
      flags[5] = 1;
    }
    else if(key == "offset_z"){
      offset_z = stof(value);
      flags[6] = 1;
    }
    else if(key == "mass"){
      mass = stof(value);
      flags[7] = 1;
    }
    else if(key == "damping"){
      damping = stof(value);
      flags[8] = 1;
    }
    else if(key == "stiffness"){
      stiffness = stof(value);
      flags[9] = 1;
    }
  }

  std::cout << "Initializing Mass Spring Damping system..." << std::endl;
  msd = std::make_shared<MassSpringDamping>(mass, stiffness, damping, false);
  t0 = std::chrono::steady_clock::now();

}

void DeformableObject::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model = _parent;

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
  msd->computePositions(grid, std::chrono::duration_cast<std::chrono::microseconds>(t - t0).count() * 1e-06);
  t0 = t;
}