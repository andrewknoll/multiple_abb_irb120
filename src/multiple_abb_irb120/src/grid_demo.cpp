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

const double JUMP_THRESHOLD = 0.0;
const double EEF_STEP = 0.01;
const std::string ROBOT_PREFIX = "abb_irb120_3_58_robot";
const std::string LINK_NAME = "link_6";
const std::string DEMO_POSE_NAME = "demo_pose";
const std::string ALL_ZERO_POSE_NAME = "all_zero";
const int NUM_ROBOTS = 2;

//Progress of the demo for synchronization of both robots
int step[2] = {0, 0};
std::mutex step_mutex;
std::condition_variable step_cv;

using MGIPtr = std::shared_ptr <moveit::planning_interface::MoveGroupInterface>;

geometry_msgs::Pose getAdjustedSpherePose(GridState& gridState, geometry_msgs::Point base_position, tf2::Quaternion o, int i, int j){
  geometry_msgs::Pose target = gridState.getPose(i, j);

  target.position.x -= base_position.x;
  target.position.y -= base_position.y;
  target.position.z -= base_position.z;

  target.orientation.x = o.x();
  target.orientation.y = o.y();
  target.orientation.z = o.z();
  target.orientation.w = o.w();

  return target;
}



void doDemo(GridState& gridState, RobotInterface& robot, int robot_i, int sphere_i, int sphere_j, bool right, bool release){
  int p;
  bool c = false;
  tf2::Quaternion facing_down;
  multiple_abb_irb120::GrabPetition grabMsg;
  std_msgs::String robot_name, link_name;
  std::vector<geometry_msgs::Pose> waypoints(5);
  geometry_msgs::Pose initial, target;
  moveit_msgs::RobotTrajectory trajectory;
  MGIPtr move_group = robot.getMoveGroup();
  ros::NodeHandle n;
  ros::Publisher grabPub = n.advertise<multiple_abb_irb120::GrabPetition>("/grid/grab_petitions", 100);

  facing_down.setRPY(0, M_PI, 0);
  move_group->setPlanningTime(10.0);
  
  std::cout << "Robot " << robot_i + 1 << ": " << "Approaching sphere " << sphere_i << " " << sphere_j << "..." << std::endl;

  //////////////////////////////////
  // Approach sphere
  //////////////////////////////////
  if(ros::ok()){
    do {
      auto A = move_group->getLinkNames();
      for(auto b : A) {
        std::cout << "Link" << std::endl;
        std::cout << b << std::endl;
      }
      target = getAdjustedSpherePose(gridState, robot.getBasePosition(), facing_down, sphere_i, sphere_j);
      std::cout << "EEF" << robot_i << " " << move_group->getEndEffectorLink() << std::endl;
      std::cout << robot_i << " " << move_group->getCurrentPose().pose << std::endl;
      move_group->setPoseTarget(target);
      move_group->move();
    } while(ros::ok() && !utils::isNear(move_group->getCurrentPose().pose, getAdjustedSpherePose(gridState, robot.getBasePosition(), facing_down, sphere_i, sphere_j), 0.04));
  }
  
  move_group->stop();
  std::cout << "Robot " << robot_i + 1 << ": " << "Approached sphere " << sphere_i << " " << sphere_j << "." << std::endl;

  //////////////////////////////////
  // Grab sphere
  //////////////////////////////////
  link_name.data = LINK_NAME;
  robot_name.data = ROBOT_PREFIX + std::to_string(robot_i + 1);
  grabMsg.robot_name = robot_name;
  
  grabMsg.i = sphere_i;
  grabMsg.j = sphere_j;
  grabMsg.link_name = link_name;
  grabMsg.robot_name = robot_name;
  grabMsg.grab = true;

  // SYNC
  step[robot_i] = 1;
  step_cv.notify_all();
  while(!c){
    c = true;
    for(int i = 0; i < NUM_ROBOTS && c; i++){
      c &= step[i] == 1;
    }
    if(!c && ros::ok()){
      std::unique_lock<std::mutex> lck(step_mutex);
      step_cv.wait_for(lck, std::chrono::milliseconds(100));
    }
    if(!ros::ok()){
      return;
    }
  }
  grabPub.publish(grabMsg);
  gridState.setGrabbed(robot_i, true);

  std::cout << "Robot " << robot_i + 1 << ": " << "Grabbed sphere " << sphere_i << " " << sphere_j << "." << std::endl;
  initial = move_group->getCurrentPose().pose;

  //////////////////////////////////
  // Move in a square
  //////////////////////////////////
  std::cout << "Robot " << robot_i << ": " << "Starting trajectory..." << std::endl;
  //Get to the initial pose
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  

  //Creating 4 positions in a square
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  for (int i = 0; i < 2; i++) { //Column
    for (int j = 0; j < 2; j++) {
      p = (i == 0 ? j : 3 - j);
      waypoints[p].position.x = initial.position.x + (i == 0 ? -0.1 : 0.1);
      waypoints[p].position.y = initial.position.y + (j == 1 ? -0.1 : 0.1);
      waypoints[p].position.z = initial.position.z;

      waypoints[p].orientation = initial.orientation;
    }
  }
  //Assign the final position to the initial
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  waypoints[4] = waypoints[0];

  //Making a square movement
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  double fraction = move_group->computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  my_plan.trajectory_ = trajectory;
  move_group->execute(my_plan);

  std::cout << "Robot " << robot_i + 1 << ": " << "Finished trajectory." << std::endl;

  // SYNC
  step[robot_i] = 2;
  step_cv.notify_all();
  while(!c){
    c = true;
    for(int i = 0; i < NUM_ROBOTS && c; i++){
      c &= step[i] == 2;
    }
    if(!c && ros::ok()){
      std::unique_lock<std::mutex> lck(step_mutex);
      step_cv.wait_for(lck, std::chrono::milliseconds(100));
    }
    if(!ros::ok()){
      return;
    }
  }
  std::cout << "Robot " << robot_i + 1 << ": " << "Starting second movement..." << std::endl;
  //////////////////////////////////
  // Deform object
  //////////////////////////////////
  target = initial;
  target.position.x += right ? -0.5 : 0.5;
  target.position.z += 0.25;

  move_group->setPositionTarget(target.position.x, target.position.y, target.position.z);
  move_group->move();
  std::cout << "Robot " << robot_i + 1 << ": " << "Finished second movement." << std::endl;
  // SYNC
  step[robot_i] = 3;
  step_cv.notify_all();
  while(!c){
    c = true;
    for(int i = 0; i < NUM_ROBOTS && c; i++){
      c &= step[i] == 3;
    }
    if(!c && ros::ok()){
      std::unique_lock<std::mutex> lck(step_mutex);
      step_cv.wait_for(lck, std::chrono::milliseconds(100));
    }
    if(!ros::ok()){
      return;
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  //////////////////////////////////
  // Release sphere by one robot
  //////////////////////////////////
  if(release){
    std::cout << "Robot " << robot_i + 1 << ": " << "Released sphere " << sphere_i << " " << sphere_j << std::endl;
    grabMsg.grab = false;
    grabPub.publish(grabMsg);
    gridState.setGrabbed(robot_i, false);
  }

  std::this_thread::sleep_for(std::chrono::seconds(3));
  //////////////////////////////////
  // Release and finish demo
  //////////////////////////////////

  if(!release){
    std::cout << "Robot " << robot_i + 1 << ": " << "Released sphere " << sphere_i << " " << sphere_j << std::endl;
    grabMsg.grab = false;
    grabPub.publish(grabMsg);
    gridState.setGrabbed(robot_i, false);
  }
  std::cout << "Robot " << robot_i + 1 << ": " << "Moving to initial pose..." << std::endl;
  move_group->setNamedTarget(ALL_ZERO_POSE_NAME);
  move_group->move();
  std::cout << "Robot " << robot_i + 1 << ": " << "Finished." << std::endl;
}

int main(int argc, char** argv)
{

  // Setup
  // ^^^^^

  std::string name_ = "robots_controller";
  ros::init(argc, argv, name_);
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();



  geometry_msgs::Point robot_bases[2];
  for(int i = 0; i < 2; i++){
    robot_bases[i].x = i;
    robot_bases[i].y = 0;
    robot_bases[i].z = 0;
  }

  RobotInterface robots[2] = {RobotInterface(robot_bases[0], "manipulator", "robot1"), RobotInterface(robot_bases[1], "manipulator", "robot2")};
  
  std::vector<std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> > planning_scene_interfaces;
  planning_scene_interfaces.push_back(std::make_shared<moveit::planning_interface::PlanningSceneInterface>("robot1"));
  planning_scene_interfaces.push_back(std::make_shared<moveit::planning_interface::PlanningSceneInterface>("robot2"));

  std::cout << "NAME1" << robots[0].getMoveGroup()->getRobotModel()->getName() << std::endl;
  std::cout << "NAME2" << robots[1].getMoveGroup()->getRobotModel()->getName() << std::endl;

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

  const int sphere_i[2] = {0, 0};
  const int sphere_j[2] = {0, resolution[1] - 1};

  GridState gridState(size, resolution, offset, sphere_radius, planning_scene_interfaces, robots, 2);

  ros::Subscriber sub = n.subscribe("/gazebo/link_states", 1000, &GridState::updateCallback, &gridState);

  while(!gridState.isReady()){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::thread t1(doDemo, std::ref(gridState), std::ref(robots[0]), 0, sphere_i[0], sphere_j[0], false, false);
  doDemo(gridState, robots[1], 1, sphere_i[1], sphere_j[1], true, true);

  t1.join();

  if(ros::ok()){
    ros::shutdown();
  }

  std::cout << "Shut down" << std::endl;

  return 0;
}
