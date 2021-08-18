#include <vector>
#include <geometry_msgs/Pose.h>
#include "gazebo/gazebo.hh"
#include <gazebo_msgs/LinkStates.h>

class GridState {
private:
    const std::string LINK_PREFIX = "grid::link_";
    std::vector<std::vector<geometry_msgs::Pose> > link_poses;
    int hres, vres;
public:
    GridState(std::vector<int> resolution);
    void updateCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
    const geometry_msgs::Pose& getPose(int i, int j) const;
};