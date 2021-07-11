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

#include "RobotInterface.hpp"

void printJointValues(moveit::planning_interface::MoveGroupInterface &move_group) {
  std::vector<std::string> joint_names = move_group.getJoints();
  std::vector<double> joint_values = move_group.getCurrentJointValues();

  for (int i = 0; i < joint_names.size(); i++) {
    std::cout << joint_names[i] << ": " << joint_values[i] << std::endl;
  }
}

void printCurrentPose(moveit::planning_interface::MoveGroupInterface &move_group) {
  geometry_msgs::PoseStamped pose = move_group.getCurrentPose();

  std::cout << pose.pose.position << std::endl
            << pose.pose.orientation << std::endl;
}

int main(int argc, char **argv)
{

  //
  // Setup
  // ^^^^^
  std::string name_ = "multiple_abb_irb120_demo";
  ros::init(argc, argv, name_);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<RobotInterface> robots = {RobotInterface("manipulator", "robot1"), RobotInterface("manipulator", "robot2")};

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface("/robot1");

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  geometry_msgs::Pose initial;
  std::vector<geometry_msgs::Pose> targets(5);
  int p;

  std::cout << "Moving Robots to 4 points in a square..." << std::endl;

  for (auto robot : robots) {
    move_group = robot.getMoveGroup();

    //Get to the initial pose
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    move_group->setNamedTarget("demo_pose");
    move_group->move();
    initial = move_group->getCurrentPose().pose;

    //Creating 4 positions in a square
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    for (int i = 0; i < 2; i++) { //Column
      for (int j = 0; j < 2; j++) {
        p = (i == 0 ? j : 3 - j);
        targets[p].position.x = initial.position.x + (i == 0 ? -0.1 : 0.1);
        targets[p].position.y = initial.position.y + (j == 1 ? -0.1 : 0.1);
        targets[p].position.z = initial.position.z;

        targets[p].orientation = initial.orientation;
      }
    }
    //Assign the final position to the initial
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    targets[4] = targets[0];

    //Moving to 4 objective positions
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    for (int p = 0; p < 4; p++) {
      move_group->setPoseTarget(targets[p]);

      move_group->move();
    }

    move_group->setPlanningTime(10.0);

    //Making a square movement
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    double fraction = move_group->computeCartesianPath(targets, eef_step, jump_threshold, trajectory);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    my_plan.trajectory_ = trajectory;
    move_group->execute(my_plan);
  }

  ros::shutdown();
  return 0;
}
