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
#include <geometry_msgs/PoseStamped.h>
#include <multiple_abb_irb120/GrabPetition.h>

using namespace gazebo;


DeformableObject::SphereGrasp::SphereGrasp(physics::LinkPtr link, ros::Subscriber sub) : link(link), sub(sub){
  updater = std::make_shared<std::thread>(&DeformableObject::SphereGrasp::updatePose, this);
}

DeformableObject::SphereGrasp::SphereGrasp(physics::LinkPtr link, std::shared_ptr<ros::NodeHandle> n, std::string topic)  : link(link) {
  sub = n->subscribe(topic, 1000, &DeformableObject::SphereGrasp::setGrabbedSpherePosCallback, this);
  updater = std::make_shared<std::thread>(&DeformableObject::SphereGrasp::updatePose, this);
}

void DeformableObject::SphereGrasp::setGrabbedSpherePosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  pose_cache = msg->pose;
  must_update = true;
  cv.notify_one();
}

void DeformableObject::SphereGrasp::updatePose() {
  while(up){
    std::unique_lock<std::mutex> lck(mtx);
    cv.wait(lck, [this]{return must_update;});
    while(must_update){
      link->SetWorldPose(toIgnitionPose3d(pose_cache));
      must_update = false;
    }
  }
}


void DeformableObject::SphereGrasp::shutdown(){
  up = false;
  updater->join();
  sub.shutdown();
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
  for(auto &s : subscribers){
    s.second->shutdown();
  }
  ros::shutdown();
  std::cout << "Deformable object plugin stopped." << std::endl;
}

void DeformableObject::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model = _parent;

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
  ros_nh.getParam("/grid/mass", mass);
  ros_nh.getParam("/grid/stiffness", stiffness);
  ros_nh.getParam("/grid/damping", damping);

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
  msd = std::make_shared<MassSpringDamping>(mass, stiffness, damping, false, this->model->GetWorld()->Gravity());

  if(!grid_initialized){
    std::cout << "Initializing grid..." << std::endl;
    grid = std::make_shared<multiple_abb_irb120::Grid>(model, offset_x, offset_y, offset_z, width, height, horizontal_res, vertical_res);
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

void DeformableObject::grabCallback(const multiple_abb_irb120::GrabPetition::ConstPtr& msg) {
  this->grid->get(msg->i, msg->j).setGrabbed(msg->grab);
  if(msg->grab){
     std::cout << "Grabbed ma ball" << std::endl;
    subscribers[msg->topic.data] = std::make_shared<SphereGrasp>(this->grid->getLink(msg->i, msg->j), this->rosNode, msg->topic.data);
  }
  else{
    subscribers[msg->topic.data]->shutdown();
    subscribers.erase(msg->topic.data);
  }
}

void DeformableObject::OnUpdate() {
  t = std::chrono::steady_clock::now();
  msd->computePositions(grid/*, std::chrono::duration_cast<std::chrono::microseconds>(t - t0).count() * 1e-06*/);
  grid->update();
  t0 = t;
}