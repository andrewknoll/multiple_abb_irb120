#include <vector>
#include "geometry_msgs/Pose.h"
#include "gazebo/gazebo.hh"
#include "gazebo_msgs/LinkStates.h"
#include "GridState.hpp"

GridState::GridState(std::vector<int> resolution) : vres(resolution[0]), hres(resolution[1]) {
    this->link_poses = std::vector<std::vector<geometry_msgs::Pose> >(vres);
    for(int i = 0; i < vres; i++){
        //Initialize column
        link_poses[i] = std::vector<geometry_msgs::Pose>(hres);
    }
}

void GridState::updateCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
    std::string name;
    int number;
    for(int l = 0; l < msg->name.size(); l++){
        name = msg->name[l];
        size_t pos = name.find(LINK_PREFIX);
        if(pos != std::string::npos){
            name.erase(pos, LINK_PREFIX.length());
            try{
                number = stoi(name);
                this->link_poses[number / hres][number % hres] = msg->pose[l];
            }
            catch(...){}
        }
    }
}

const geometry_msgs::Pose& GridState::getPose(int i, int j) const{
    return this->link_poses[i][j];
}
