/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */
// MODIFIED BY: Andrés Otero García, Gonzalo López Nicolás and María del Rosario Aragüés Muñoz


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <iostream>     // std::streambuf, std::cout
#include <fstream>      // std::ofstream

#include <ros/service_client.h>
#include <ros/ros.h>

#include "RobotInterface.hpp"
#include "GridState.hpp"
#include <chrono>

#include <signal.h>
#include <chrono>

#include "utils.hpp"

#include <multiple_abb_irb120/GrabPetition.h>

#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>

bool volatile shutdown_request = false;

void signalHandler( int signum ) {
  std::cout << "Interrupt received: " << signum << std::endl;

  // cleanup and close up stuff here  
  // terminate program  

  shutdown_request = true;
}

int main(int argc, char** argv)
{
  //signal(SIGINT, signalHandler);
  //signal(SIGKILL, signalHandler);
  //signal(SIGTERM, signalHandler);
  //
  // Setup
  // ^^^^^

if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
}

  std::string name_ = "robots_controller";
  ros::init(argc, argv, name_);
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  


  RobotInterface robots[2] = {RobotInterface("manipulator", "robot1"), RobotInterface("manipulator", "robot2")};
  
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>("/robot1");

  std::shared_ptr <moveit::planning_interface::MoveGroupInterface> move_group;

  ros::Rate loop_rate(10);

  // Start the Grid
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  std::vector<double> size(2);
  ros::param::get("/grid/width", size[0]);
  ros::param::get("/grid/height", size[1]);
  std::vector<int> resolution;
  ros::param::get("/grid/resolution", resolution);
  std::vector<double> offset;
  ros::param::get("/grid/offset", offset);
  float sphere_radius = 0.025;
  ros::param::get("/grid/sphere_radius", sphere_radius);

  move_group = robots[0].getMoveGroup();

  std::cout << "insert generic comment here" << std::endl;
  GridState gridState(size, resolution, offset, sphere_radius, planning_scene_interface, move_group->getPlanningFrame());

  ros::Subscriber sub = n.subscribe("/gazebo/link_states", 1000, &GridState::updateCallback, &gridState);
  ros::Publisher grabPub = n.advertise<multiple_abb_irb120::GrabPetition>("/grid/grab_petitions", 1000);
  multiple_abb_irb120::GrabPetition grabMsg;

  geometry_msgs::Pose target_pose;

  /*while(ros::ok() && !utils::isNear(move_group->getCurrentPose().pose, gridState.getPose(0, 0), 0.04)){
    //std::cout << "Sphere pose: " << gridState.getPose(0, 0).position.x << " " << gridState.getPose(0, 0).position.y << " " << gridState.getPose(0, 0).position.z << std::endl;
    if(target_pose != gridState.getPose(0, 0)){
      target_pose = gridState.getPose(0, 0);
      move_group->setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);
      move_group->asyncMove();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }*/

  while(!gridState.isReady()){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  auto asdnfoaingoaiernoiaenrgoiang = utils::toIgnitionPose3d(move_group->getCurrentPose().pose);
  auto asdf = asdnfoaingoaiernoiaenrgoiang.Rot();
  std::cout << "asdf: " << asdf.Roll() << " " << asdf.Pitch() << " " << asdf.Yaw() << std::endl;

  while(ros::ok() && !utils::isNear(move_group->getCurrentPose().pose, gridState.getPose(0, 0), 0.04)){
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, M_PI, 0);
    target_pose = gridState.getPose(0, 0);
    target_pose.orientation.x = myQuaternion.x();
    target_pose.orientation.y = myQuaternion.y();
    target_pose.orientation.z = myQuaternion.z();
    target_pose.orientation.w = myQuaternion.w();
    move_group->setPoseTarget(target_pose);
    //move_group->setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);
    //move_group->setOrientationTarget(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
    move_group->move();
  }

  move_group->stop();

  std_msgs::String link_name;
  link_name.data = "abb_irb120_3_58_robot1::link_6";
  grabMsg.link_name = link_name;
  grabMsg.i = 0;
  grabMsg.j = 0;
  grabMsg.grab = true;

  gridState.setGrabbed(0, 0, true);
  grabPub.publish(grabMsg);

  std::cout << "Tuto bene" << std::endl;

  int positions = 0;
  while(ros::ok() && positions < 4){
    move_group->setRandomTarget();
    move_group->move();
    positions++;
  }

  /*for(auto& robot : robots){
    std::cout << "aoignaoedrnaoehnpsanhpahgahagfadastananf" << std::endl;
    robot.shutdown();
  }*/

  grabMsg.grab = false;
  grabPub.publish(grabMsg);

  if(ros::ok()){
    ros::shutdown();
  }

  std::cout << "Shut down" << std::endl;

  return 0;
}
