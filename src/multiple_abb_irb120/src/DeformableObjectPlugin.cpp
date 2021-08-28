#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "DeformableObjectPlugin.hpp"
#include "Grid.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <ros/package.h>
#include "MassSpringDamping.hpp"
#include "utils.hpp"
#include <multiple_abb_irb120/GrabPetition.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>

using namespace gazebo;

/*
DeformableObject::SphereGrasp::SphereGrasp(physics::LinkPtr link, double radius, ros::Subscriber sub) : link(link), RADIUS(radius), sub(sub){
  //updater = std::make_shared<std::thread>(&DeformableObject::SphereGrasp::updatePose, this);
}

DeformableObject::SphereGrasp::SphereGrasp(physics::LinkPtr link, double radius, std::shared_ptr<ros::NodeHandle> n, std::string end_effector_name)  : link(link), RADIUS(radius), link_name(end_effector_name) {
  //sub = n->subscribe("/gazebo/link_states", 1000, &DeformableObject::SphereGrasp::setGrabbedSpherePosCallback, this);
}
*/
DeformableObject::SphereGrasp::SphereGrasp(physics::LinkPtr link, double radius, physics::WorldPtr world, std::string robot_name, std::string end_effector_name)  : link(link), RADIUS(radius), world(world), robot_name(robot_name), link_name(end_effector_name) {
  updater = std::make_shared<std::thread>(&DeformableObject::SphereGrasp::updatePose, this);
}

void DeformableObject::SphereGrasp::updatePose(){
  while(up){
    physics::ModelPtr model = world->ModelByName(robot_name);
    if(model != nullptr){
      physics::LinkPtr robot_link = model->GetLink(link_name);
      if(robot_link != nullptr){
        ignition::math::Pose3d offset(RADIUS + EPSILON, 0, 0, 0, 0, 0);
        link->SetWorldPose(offset + robot_link->WorldPose());
      }
    }
  }
}


/*
void DeformableObject::SphereGrasp::setGrabbedSpherePosCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
  std::string name;
   ignition::math::Pose3d offset(0, 0, RADIUS + EPSILON, 0, 0, 0);
  for(int l = 0; l < msg->name.size(); l++){
    name = msg->name[l];
    size_t pos = name.find(this->link_name);
    if(pos != std::string::npos){
      link->SetWorldPose(offset + utils::toIgnitionPose3d(msg->pose[l]));
      break;
    }
  }
}
*/

void DeformableObject::SphereGrasp::shutdown(){
  if(up){
    up = false;
    updater->join();
    //sub.shutdown();
  }
}

DeformableObject::SphereGrasp::~SphereGrasp(){
  shutdown();
}

DeformableObject::DeformableObject() : ModelPlugin()
{
  std::cout << "Starting deformable object plugin..." << std::endl;
  t0 = std::chrono::steady_clock::now();

}
DeformableObject::~DeformableObject(){
  for(auto &s : grasps){
    s.second->shutdown();
  }
  ros::shutdown();
  std::cout << "Deformable object plugin stopped." << std::endl;
}

void DeformableObject::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model = _parent;
  this->world = _parent->GetWorld();

  if(_sdf->HasElement("testing")){
    testing = _sdf->Get<bool>("testing");
  }

  const std::string PACKAGE_PATH = ros::package::getPath("multiple_abb_irb120");
  std::vector<int> resolution;
  std::vector<double> offset;

  ros::NodeHandle ros_nh;
  ros_nh.getParam("/grid/width", width);
  ros_nh.getParam("/grid/height", height);
  
  ros_nh.getParam("/grid/resolution", resolution);
  vertical_res = resolution[0];
  horizontal_res = resolution[1];
  ros_nh.getParam("/grid/offset", offset);
  offset_x = offset[0];
  offset_y = offset[1];
  offset_z = offset[2];
  ros_nh.getParam("/grid/sphere_radius", radius);
  ros_nh.getParam("/grid/mass", mass);
  ros_nh.getParam("/grid/stiffness", stiffness);
  ros_nh.getParam("/grid/damping", damping);
  ros_nh.getParam("/grid/gravity", gravity);

  std::cout << "Initializing Mass Spring Damping system..." << std::endl;
  std::cout << "Parameters: " << std::endl;
  std::cout << "width: " << width << std::endl;
  std::cout << "height: " << height << std::endl;
  std::cout << "vertical_res: " << vertical_res << std::endl;
  std::cout << "horizontal_res: " << horizontal_res << std::endl;
  std::cout << "offset_x: " << offset_x << std::endl;
  std::cout << "offset_y: " << offset_y << std::endl;
  std::cout << "offset_z: " << offset_z << std::endl;
  std::cout << "radius: " << radius << std::endl;
  std::cout << "mass: " << mass << std::endl;
  std::cout << "stiffness: " << stiffness << std::endl;
  std::cout << "damping: " << damping << std::endl;
  msd = std::make_shared<MassSpringDamping>(mass, stiffness, damping, gravity, this->model->GetWorld()->Gravity());

  if(!grid_initialized){
    std::cout << "Initializing grid..." << std::endl;
    grid = std::make_shared<multiple_abb_irb120::Grid>(model, offset_x, offset_y, offset_z, width, height, horizontal_res, vertical_res, testing);
    std::cout << "Grid: Success!" << std::endl;
  }

  // Listen to the update event. This event is broadcast every
      // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&DeformableObject::OnUpdate, this));


  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "deformable_object_client",
        ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("deformable_object_client"));

  // Create a named topic, and subscribe to it.
  this->grabSub = this->rosNode->subscribe("/grid/grab_petitions", 1000, &DeformableObject::grabCallback, this);

  std::cout << "Deformable Object Plugin Initialized." << std::endl;
}
/*
void DeformableObject::getRobotNameAndLink(std::string full_name, std::string& robot_name, std::string& link_name, std::string delimiter){
  size_t pos = full_name.find(delimiter);
  if(pos != std::string::npos){
    robot_name = full_name.substr(0, pos);
    link_name = full_name.substr(pos);
    link_name.erase(0, delimiter.length());
    std::cout << "Link you asked: " << robot_name << " " << link_name << std::endl;
  }
}
*/
void DeformableObject::grabCallback(const multiple_abb_irb120::GrabPetition::ConstPtr& msg) {
  this->grid->get(msg->i, msg->j).setGrabbed(msg->grab);
  if(msg->grab){
    std::string robot_name, link_name;
    grasps[{msg->i, msg->j}] = std::make_shared<SphereGrasp>(this->grid->getLink(msg->i, msg->j), this->radius, this->world, msg->robot_name.data, msg->link_name.data);
  }
  else{
    grasps[{msg->i, msg->j}]->shutdown();
    grasps.erase({msg->i, msg->j});
  }
}

void DeformableObject::OnUpdate() {
  t = std::chrono::steady_clock::now();
  msd->computePositions(grid);
  grid->update();
  t0 = t;
}