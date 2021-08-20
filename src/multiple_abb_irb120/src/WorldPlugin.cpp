//Gazebo v9.0.0
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
//#include "GazeboPlugin.hpp"
#include "ros/ros.h"
#include "ros/package.h"
#include "utils.hpp"
#include <chrono> 

const std::string MODEL_PLUGIN_NAME = "gazebo_plugin";
const std::string PACKAGE_NAME = "multiple_abb_irb120";

double calculateInitialComponent(int index, double offset, double size, int resolution) {
    return offset + (double)(index - (double)(resolution - 1) / 2) * (size / (double)resolution);
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

gazebo::msgs::Material* calculateMaterial(int indices[3], int resolution[3]){
    gazebo::msgs::Material* material = new gazebo::msgs::Material();
    gazebo::msgs::Color* color = new gazebo::msgs::Color();
    
    color->set_r((resolution[1] - indices[1]) / (double)resolution[1]);
    color->set_g(indices[0] / (double)resolution[0]);
    color->set_b((resolution[0] - indices[0]) / (double)resolution[0]);;
    color->set_a(1.0);
    std::cout << "color: " << color->r() << " " << color->g() << " " << color->b() << " " << color->a() << std::endl;
    material->set_allocated_ambient(color);
    return material;
}

std::string fromFileToString(std::string filename) {
  std::ifstream f(filename);
  std::stringstream buffer;
  buffer << f.rdbuf();

  return buffer.str();
}

namespace gazebo
{
class Factory : public WorldPlugin
{
  physics::WorldPtr world;
  float width = 5.0, height = 5.0;
  int vertical_res = 10, horizontal_res = 10;
  float offset_x = 0.0, offset_y = 0.0, offset_z = 1.0;
  float mass = 0.1, stiffness = 20.0, damping = 2.0;
  float radius = 0.025;


  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    const std::string PACKAGE_PATH = ros::package::getPath(PACKAGE_NAME);
    std::map<std::string, double> parameters = readParameters(PACKAGE_PATH + "/config/grid.config");
    if(parameters.count("width") != 0) width = parameters["width"];
    if(parameters.count("height") != 0) height = parameters["height"];
    if(parameters.count("vertical_res") != 0) vertical_res = (int)parameters["vertical_res"];
    if(parameters.count("horizontal_res") != 0) horizontal_res = (int)parameters["horizontal_res"];
    if(parameters.count("offset_x") != 0) offset_x = parameters["offset_x"];
    if(parameters.count("offset_y") != 0) offset_y = parameters["offset_y"];
    if(parameters.count("offset_z") != 0) offset_z = parameters["offset_z"];
    if(parameters.count("radius") != 0) radius = parameters["radius"];
    if(parameters.count("mass") != 0) mass = parameters["mass"];
    if(parameters.count("stiffness") != 0) stiffness = parameters["stiffness"];
    if(parameters.count("damping") != 0) damping = parameters["damping"];

    ros::NodeHandle ros_nh;
    ros_nh.setParam("/grid/width", width);
    ros_nh.setParam("/grid/height", height);
    ros_nh.setParam("/grid/resolution", std::vector<int>({vertical_res, horizontal_res}));
    ros_nh.setParam("/grid/offset", std::vector<double>({offset_x, offset_y, offset_z}));
    ros_nh.setParam("/grid/sphere_radius", radius);
    ros_nh.setParam("/grid/mass", mass);
    ros_nh.setParam("/grid/stiffness", stiffness);
    ros_nh.setParam("/grid/damping", damping);
        
    this->world = _parent;

    std::string suffix;

    sdf::SDFPtr modelSDF(new sdf::SDF);
    sdf::init(modelSDF);

    gazebo::msgs::Model model;
    model.set_name("grid");

    //Add Model Plugin (models the behaviour of the cloth)
    gazebo::msgs::Plugin* plugin = model.add_plugin();
    plugin->set_name(MODEL_PLUGIN_NAME);
    plugin->set_filename("lib" + MODEL_PLUGIN_NAME + ".so");
    //If it was necessary, a visual plugin may be added here

    double offset[3] = {offset_x, offset_y, offset_z};
    double size[3] = {width, height, 1};
    int resolution[3] = {horizontal_res, vertical_res, 1};
    int indices[3] = {0, 0, 0};

    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    std::chrono::duration<double> elapsed;

    //Spawn all links with their respective collision spheres
    for(int i = 0; i < vertical_res; i++){
      indices[1] = i;
      for(int j = 0; j < horizontal_res; j++){
        indices[0] = j;
        printf("Creating link...\n");
        start = std::chrono::high_resolution_clock::now();
        suffix = "_" + std::to_string(i * horizontal_res + j);

        //Create a new link
        gazebo::msgs::AddSphereLink(model, mass, radius);
        gazebo::msgs::Link* link = model.mutable_link(model.link_size()-1);
        //link->set_kinematic(true);

        gazebo::msgs::Pose* pos = calculateInitialPos(indices, offset, size, resolution);

        link->set_name("link" + suffix);
        link->set_allocated_pose(pos);
        end = std::chrono::high_resolution_clock::now();
        elapsed += end - start;
        gazebo::msgs::Visual* visual = link->mutable_visual(link->visual_size()-1);
        gazebo::msgs::Material* material = calculateMaterial(indices, resolution);
        visual->set_allocated_material(material);
      }
    }
    start = std::chrono::high_resolution_clock::now();
    modelSDF->Root()->InsertElement(gazebo::msgs::ModelToSDF(model));

    _parent->InsertModelSDF(*modelSDF);
    end = std::chrono::high_resolution_clock::now();
    elapsed += end - start;

    printf("Started World Grid Plugin in %f\n", elapsed);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}