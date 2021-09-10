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

#include <cstdarg>

#include "Demo.hpp"

const int NUM_ROBOTS = 2;

using MGIPtr = std::shared_ptr <moveit::planning_interface::MoveGroupInterface>;
using GridStatePtr = GridState*;

class GridDemo : public Demo {

private:

  ros::NodeHandle n;
  ros::Publisher grabPub;

  virtual void doDemo(RobotInterface& robot, int robot_i, int params, ...) override {
    if(params != 5){
      ROS_ERROR("Wrong number of parameters for grid demo");
      return;
    }
    else{
      va_list args;
      va_start(args, params);

      GridStatePtr gridState = va_arg(args, GridStatePtr);
      int sphere_i = va_arg(args, int);
      int sphere_j = va_arg(args, int);
      bool right = va_arg(args, int);
      bool release = va_arg(args, int);

      int p;
      bool c = false;
      tf2::Quaternion facing_down;
      multiple_abb_irb120::GrabPetition grabMsg;
      std_msgs::String robot_name, link_name;
      std::vector<geometry_msgs::Pose> waypoints(5);
      geometry_msgs::Pose initial, target;
      moveit_msgs::RobotTrajectory trajectory;
      MGIPtr move_group = robot.getMoveGroup();

      facing_down.setRPY(0, M_PI, 0);
      move_group->setPlanningTime(10.0);
      
      std::cout << "Robot " << robot_i + 1 << ": " << "Approaching sphere " << sphere_i << " " << sphere_j << "..." << std::endl;

      //////////////////////////////////
      // Approach sphere
      //////////////////////////////////
      if(ros::ok()){
        do {
          target = utils::getAdjustedSpherePose(gridState->getPose(sphere_i, sphere_j), robot.getBasePosition(), facing_down);
          move_group->setPoseTarget(target);
          move_group->move();
        } while(ros::ok() && !utils::isNear(move_group->getCurrentPose().pose, utils::getAdjustedSpherePose(gridState->getPose(sphere_i, sphere_j), robot.getBasePosition(), facing_down), 0.04));
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
      if(!syncRobots(robot_i, 1)){
        va_end(args);
        return;
      }

      grabPub.publish(grabMsg);
      gridState->setGrabbed(robot_i, true);

      std::cout << "Robot " << robot_i + 1 << ": " << "Grabbed sphere " << sphere_i << " " << sphere_j << "." << std::endl;
      initial = move_group->getCurrentPose().pose;

      //////////////////////////////////
      // Move in a square
      //////////////////////////////////
      std::cout << "Robot " << robot_i << ": " << "Starting trajectory..." << std::endl;
      

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
      if(!syncRobots(robot_i, 2)){
        va_end(args);
        return;
      }
      std::cout << "Robot " << robot_i + 1 << ": " << "Starting second movement..." << std::endl;
      //////////////////////////////////
      // Deform object
      //////////////////////////////////
      target = initial;
      target.position.y += right ? 0.4 : -0.4;

      move_group->setPoseTarget(target);
      move_group->move();
      std::cout << "Robot " << robot_i + 1 << ": " << "Finished second movement." << std::endl;
      // SYNC
      if(!syncRobots(robot_i, 3)) {
        va_end(args);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      
      //////////////////////////////////
      // Release sphere by one robot
      //////////////////////////////////
      if(release){
        std::cout << "Robot " << robot_i + 1 << ": " << "Released sphere " << sphere_i << " " << sphere_j << std::endl;
        grabMsg.grab = false;
        grabPub.publish(grabMsg);
        gridState->setGrabbed(robot_i, false);
      }

      std::this_thread::sleep_for(std::chrono::seconds(3));
      //////////////////////////////////
      // Release and finish demo
      //////////////////////////////////

      if(!release){
        std::cout << "Robot " << robot_i + 1 << ": " << "Released sphere " << sphere_i << " " << sphere_j << std::endl;
        grabMsg.grab = false;
        grabPub.publish(grabMsg);
        gridState->setGrabbed(robot_i, false);
      }
      std::this_thread::sleep_for(std::chrono::seconds(2));
      std::cout << "Robot " << robot_i + 1 << ": " << "Moving to initial pose..." << std::endl;
      move_group->setNamedTarget(ALL_ZERO_POSE_NAME);
      move_group->move();
      std::cout << "Robot " << robot_i + 1 << ": " << "Finished." << std::endl;

      va_end(args);
    }
  }

public:
  GridDemo(int n_robots) : Demo(n_robots) {
    grabPub = n.advertise<multiple_abb_irb120::GrabPetition>("/grid/grab_petitions", 100);
  }

  void execute(RobotInterface& robot, int robot_i, GridState& gridState, int sphere_i, int sphere_j, bool right, bool release){
    doDemo(robot, robot_i, 5, &gridState, sphere_i, sphere_j, right, release);
  }

};

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

  GridDemo demo(NUM_ROBOTS);

  std::thread t1(&GridDemo::execute, &demo, std::ref(robots[0]), 0, std::ref(gridState), sphere_i[0], sphere_j[0], false, false);
  demo.execute(robots[1], 1, gridState, sphere_i[1], sphere_j[1], true, true);

  t1.join();

  if(ros::ok()){
    ros::shutdown();
  }

  std::cout << "Shut down" << std::endl;

  return 0;
}
