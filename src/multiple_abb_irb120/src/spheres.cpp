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
#include <multiple_abb_irb120/GrabPosition.h>

#include <ros/console.h>

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
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface("/robot1");
  for(auto& o : planning_scene_interface.getKnownObjectNames()) {
    std::cout << "Object: " << o << std::endl;
  }

  std::shared_ptr <moveit::planning_interface::MoveGroupInterface> move_group;

  ros::Rate loop_rate(10);

  // Start the Grid
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  std::vector<int> resolution;
  ros::param::get("/grid/resolution", resolution);
  GridState gridState(resolution);

  ros::Subscriber sub = n.subscribe("/gazebo/link_states", 1000, &GridState::updateCallback, &gridState);
  ros::Publisher grabPub = n.advertise<multiple_abb_irb120::GrabPetition>("/grid/grab_petitions", 1000);
  multiple_abb_irb120::GrabPetition grabMsg;


  move_group = robots[0].getMoveGroup();

  geometry_msgs::Pose target_pose;

  while(ros::ok() && !isNear(move_group->getCurrentPose().pose, gridState.getPose(0, 0), 0.1)){
    //std::cout << "Sphere pose: " << gridState.getPose(0, 0).position.x << " " << gridState.getPose(0, 0).position.y << " " << gridState.getPose(0, 0).position.z << std::endl;
    if(target_pose != gridState.getPose(0, 0)){
      target_pose = gridState.getPose(0, 0);
      move_group->setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);
      move_group->asyncMove();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  move_group->stop();

  std_msgs::String topicName;
  topicName.data = "/robot1/end_effector_pose";
  grabMsg.topic = topicName;
  grabMsg.i = 0;
  grabMsg.j = 0;
  grabMsg.grab = true;

  grabPub.publish(grabMsg);

  std::cout << "Tuto bene" << std::endl;

  while(ros::ok()){
    move_group->setRandomTarget();
    move_group->move();
  }

  for(auto& robot : robots){
    std::cout << "aoignaoedrnaoehnpsanhpahgahagfadastananf" << std::endl;
    robot.shutdown();
  }

  ros::shutdown();
  return 0;
}
