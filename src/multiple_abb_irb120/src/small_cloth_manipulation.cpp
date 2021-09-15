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

#include "Demo.hpp"


const int NUM_ROBOTS = 2;

using MGIPtr = std::shared_ptr <moveit::planning_interface::MoveGroupInterface>;
using GridStatePtr = GridState*;

class SmallClothDemo : public Demo {
  
private:

  ros::NodeHandle n;
  ros::Publisher grabPub;

  virtual void doDemo(RobotInterface& robot, int robot_i, int params, ...) override {
    if(params != 3){
      ROS_ERROR("Wrong number of parameters for grid demo");
      return;
    }
    else{
      va_list args;
      va_start(args, params);

      GridStatePtr gridState = va_arg(args, GridStatePtr);
      int sphere_i = va_arg(args, int);
      int sphere_j = va_arg(args, int);

      tf2::Quaternion facing_down;
      multiple_abb_irb120::GrabPetition grabMsg;
      std_msgs::String robot_name, link_name;
      std::vector<geometry_msgs::Pose> waypoints(1);
      geometry_msgs::Pose initial, target, sphere_initial;
      moveit_msgs::RobotTrajectory trajectory;
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      MGIPtr move_group = robot.getMoveGroup();

      facing_down.setRPY(0, M_PI, 0);
      move_group->setPlanningTime(10.0);

      initial = move_group->getCurrentPose().pose;
      sphere_initial = utils::getAdjustedSpherePose(gridState->getPose(sphere_i, sphere_j), robot.getBasePosition(), facing_down);
      
      std::cout << "Robot " << robot_i + 1 << ": " << "Approaching sphere " << sphere_i << " " << sphere_j << "..." << std::endl;

      //////////////////////////////////
      // Approach sphere
      //////////////////////////////////
      if(ros::ok()){
        do {
          target = utils::getAdjustedSpherePose(gridState->getPose(sphere_i, sphere_j), robot.getBasePosition(), facing_down);
          
          move_group->setPoseTarget(target);
          move_group->move();
        } while(ros::ok() && !utils::isNear(move_group->getCurrentPose().pose, utils::getAdjustedSpherePose(gridState->getPose(sphere_i, sphere_j), robot.getBasePosition(), facing_down), 0.03));
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
      if(!syncRobots(robot_i, 1)) {
          va_end(args);
          return;
        }

      grabPub.publish(grabMsg);
      gridState->setGrabbed(robot_i, true);

      std::cout << "Robot " << robot_i + 1 << ": " << "Grabbed sphere " << sphere_i << " " << sphere_j << "." << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      //////////////////////////////////
      // Go up
      //////////////////////////////////
      std::cout << "Robot " << robot_i << ": " << "Going up..." << std::endl;

      //Get a position above the sphere
      target = move_group->getCurrentPose().pose;
      target.position.z = sphere_initial.position.z + 0.3;

      move_group->setPoseTarget(target);
      move_group->move();

      waypoints[0] = move_group->getCurrentPose().pose;
      waypoints[0].position.x = 0;
      waypoints[0].position.z += 0.1;

      move_group->computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory);

      my_plan.trajectory_ = trajectory;
      move_group->execute(my_plan);


      std::cout << "Robot " << robot_i + 1 << ": " << "Finished going up." << std::endl;

      // SYNC
      if(!syncRobots(robot_i, 2)) {
        va_end(args);
        return;
      }

      std::cout << "Robot " << robot_i + 1 << ": " << "Starting shaking..." << std::endl;
      //////////////////////////////////
      // Shake object
      //////////////////////////////////
      initial = move_group->getCurrentPose().pose;

      waypoints[0].position.y = initial.position.y;
      waypoints[0].position.z = initial.position.z;

      waypoints[0].orientation.x = facing_down.x();
      waypoints[0].orientation.y = facing_down.y();
      waypoints[0].orientation.z = facing_down.z();
      waypoints[0].orientation.w = facing_down.w();

      for (int i = 0; i < 3; i++) {
        waypoints[0].position.x = initial.position.x + (i % 2? 0.2 : -0.2);

        move_group->computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory);

        my_plan.trajectory_ = trajectory;
        move_group->execute(my_plan);

        // SYNC
        if(!syncRobots(robot_i, 3 + i)) {
          va_end(args);
          return;
        }
      }
      

      std::cout << "Robot " << robot_i + 1 << ": " << "Finished shaking." << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      //////////////////////////////////
      //////// Stretch object //////////
      //////////////////////////////////
      std::cout << "Robot " << robot_i + 1 << ": " << "Starting stretching..." << std::endl;

      waypoints[0] = move_group->getCurrentPose().pose;
      waypoints[0].position.x = sphere_initial.position.x - 0.1;

      move_group->computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory);

      my_plan.trajectory_ = trajectory;
      move_group->execute(my_plan);

      target = move_group->getCurrentPose().pose;

      target.position.y += robot_i % 2 ? 0.2 : -0.2;
      std::cout << robot_i << " " << target.position.y << std::endl;
      move_group->setPoseTarget(target);
      move_group->move();

      std::cout << "Robot " << robot_i + 1 << ": " << "Finished stretching." << std::endl;
      //SYNC
      if(!syncRobots(robot_i, 6)) {
          va_end(args);
          return;
        }

      //////////////////////////////////
      //////// Go drop object //////////
      //////////////////////////////////
      std::cout << "Robot " << robot_i + 1 << ": " << "Starting drop trajectory..." << std::endl;
      waypoints[0].position.x = sphere_initial.position.x - 0.15;
      waypoints[0].position.y = sphere_initial.position.y;
      waypoints[0].position.z = sphere_initial.position.z + 0.2;
      
      target.position.x = sphere_initial.position.x + 0.15;
      target.position.y = sphere_initial.position.y;
      target.position.z = sphere_initial.position.z;

      waypoints.push_back(target);

      move_group->computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory);

      my_plan.trajectory_ = trajectory;
      move_group->execute(my_plan);

      //SYNC
      syncRobots(robot_i, 7);
      
      //////////////////////////////////
      // Release sphere by both robots and finish demo
      //////////////////////////////////

      std::cout << "Robot " << robot_i + 1 << ": " << "Released sphere " << sphere_i << " " << sphere_j << std::endl;
      grabMsg.grab = false;
      grabPub.publish(grabMsg);
      gridState->setGrabbed(robot_i, false);

      move_group->setNamedTarget(ALL_ZERO_POSE_NAME);
      move_group->move();

      va_end(args);

    }
  }

public:
  SmallClothDemo(int n_robots) : Demo(n_robots) {
    grabPub = n.advertise<multiple_abb_irb120::GrabPetition>("/grid/grab_petitions", 100);
  }

  void execute(RobotInterface& robot, int robot_i, GridState& gridState, int sphere_i, int sphere_j){
    doDemo(robot, robot_i, 3, &gridState, sphere_i, sphere_j);
  }

};

int main(int argc, char** argv){

  // Setup
  // ^^^^^

  std::string name_ = "robots_controller";
  ros::init(argc, argv, name_);
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();



  geometry_msgs::Point robot_bases[2];
  for(int i = 0; i < 2; i++){
    robot_bases[i].x = 0;
    robot_bases[i].y = i;
    robot_bases[i].z = 0;
  }

  RobotInterface robots[2] = {RobotInterface(robot_bases[0], "manipulator", "robot1"), RobotInterface(robot_bases[1], "manipulator", "robot2")};
  
  std::vector<std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> > planning_scene_interfaces;
  planning_scene_interfaces.push_back(std::make_shared<moveit::planning_interface::PlanningSceneInterface>("robot1"));
  planning_scene_interfaces.push_back(std::make_shared<moveit::planning_interface::PlanningSceneInterface>("robot2"));

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

  const int sphere_i[2] = {0, resolution[0] - 1};
  const int sphere_j[2] = {0, 0};

  GridState gridState(size, resolution, offset, sphere_radius, planning_scene_interfaces, robots, 2);

  ros::Subscriber sub = n.subscribe("/gazebo/link_states", 1000, &GridState::updateCallback, &gridState);

  while(!gridState.isReady()){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  SmallClothDemo demo(NUM_ROBOTS);

  std::thread t1(&SmallClothDemo::execute, &demo, std::ref(robots[1]), 1, std::ref(gridState), sphere_i[1], sphere_j[1]);
  demo.execute(robots[0], 0, gridState, sphere_i[0], sphere_j[0]);

  t1.join();

  if(ros::ok()){
    ros::shutdown();
  }

  std::cout << "Shut down" << std::endl;

  return 0;
}