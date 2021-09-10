#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include "RobotInterface.hpp"
#include <ros/ros.h>

class Demo {
protected:
  const double JUMP_THRESHOLD = 0.0;
  const double EEF_STEP = 0.01;
  const std::string ROBOT_PREFIX = "abb_irb120_3_58_robot";
  const std::string LINK_NAME = "link_6";
  const std::string DEMO_POSE_NAME = "demo_pose";
  const std::string ALL_ZERO_POSE_NAME = "all_zero";
  const int N;

  //Progress of the demo for synchronization of both robots
  std::vector<int> current_step;
  std::mutex step_mutex;
  std::condition_variable step_cv;

  bool syncRobots(int robot_i, int step, int tries = 100, int milliseconds = 500){
    bool c = false;
    int i = 0;
    current_step[robot_i] = step;
    step_cv.notify_all();
    while(!c && i < tries){
      c = true;
      for(int i = 0; i < N && c; i++){
        c &= current_step[i] == step;
      }
      if(!c && ros::ok()){
        std::unique_lock<std::mutex> lck(step_mutex);
        step_cv.wait_for(lck, std::chrono::milliseconds(milliseconds));
      }
      if(!ros::ok()){
        return false;
      }
      i++;
    }
    return i < tries;
  }

  Demo(int n_robots) : N(n_robots) {
    current_step = std::vector<int>(N, 0);
  }

  virtual void doDemo(RobotInterface& robot, int robot_i, int params, ...) = 0;
};