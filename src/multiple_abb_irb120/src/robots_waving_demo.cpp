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

#include <iostream>     // std::streambuf, std::cout
#include <fstream>      // std::ofstream

#include <ros/service_client.h>
#include <ros/ros.h>

#include "RobotInterface.hpp"
#include "Demo.hpp"
#include <chrono>

#include <signal.h>
#include <chrono>

#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>

const int PI_2 = M_PI / 2;
const std::string ROBOT_PREFIX = "robot";
const std::string GROUP_NAME = "manipulator";

using MGIPtr = std::shared_ptr <moveit::planning_interface::MoveGroupInterface>;

class RobotsWavingDemo : public Demo {

private:

  ros::NodeHandle n;
  ros::Publisher grabPub;

  void setup(RobotInterface& robot, int robot_i){
    MGIPtr mgi = robot.getMoveGroup();

    geometry_msgs::Pose pose = mgi->getCurrentPose().pose;
    pose.position.z = sin(robot_i * PI_2) / 4.0 + 0.5;
    std::cout << "Robot " << robot_i << " on " << pose.position.z << std::endl;

    mgi->setPoseTarget(pose);
    mgi->move();
  }

  virtual void doDemo(RobotInterface& robot, int robot_i, int params, ...) override {
    va_list args;
    va_start(args, params);

    MGIPtr mgi = robot.getMoveGroup();
    geometry_msgs::Pose pose = mgi->getCurrentPose().pose;
    for(int i = 0; i < 5; i++){
      pose.position.z = sin((robot_i + i) * PI_2) / 4.0 + 0.5;

      mgi->setPoseTarget(pose);
      mgi->move();

      syncRobots(robot_i, i + 1);
    }
  }

public:
  RobotsWavingDemo(int robots) : Demo(robots) {}

  void execute(RobotInterface& robot, int robot_i){
    setup(robot, robot_i);
    if(!syncRobots(robot_i, 1)) return;
    doDemo(robot, robot_i, 0);
  }
};

int main(int argc, char** argv)
{

  // Setup
  // ^^^^^

  std::string name_ = "robots_controller";
  ros::init(argc, argv, name_);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int n = -1;
  ros::param::param<int>("~robots", n, -1);

  if(n < 0){
    ROS_ERROR("Usage: %s <number_of_robots>", argv[0]);
    return -1;
  }

  std::vector<RobotInterface> robots;
  std::vector<geometry_msgs::Point> robot_bases(n);
  std::vector<std::thread> threads;
  

  for(int i = 0; i < n; i++){
    robot_bases[i].x = 0;
    robot_bases[i].y = i;
    robot_bases[i].z = 0;
    robots.emplace_back(robot_bases[i], GROUP_NAME, ROBOT_PREFIX + std::to_string(i + 1));
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  //////////////////////////////////
  /////// SETUP ////////////////////
  //////////////////////////////////
  RobotsWavingDemo demo(n);

  for(int i = 1; i < n; i++){
    threads.push_back(std::thread(&RobotsWavingDemo::execute, &demo, std::ref(robots[i]), i));
  }
  demo.execute(robots[0], 0);

  for(int i = 1; i < n; i++){
    threads[i-1].join();
  }

  if(ros::ok()){
    ros::shutdown();
  }

  std::cout << "Shut down" << std::endl;

  return 0;
}
