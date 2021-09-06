#include <vector>
#include <geometry_msgs/Pose.h>
#include "gazebo/gazebo.hh"
#include <gazebo_msgs/LinkStates.h>
#include <mutex>
#include <condition_variable>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <thread>
#include "RobotInterface.hpp"

class GridState {
private:
    using PSIPtr = std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>;
    std::mutex mtx;
    std::condition_variable cv;
    volatile bool must_update = false;

    enum GrabbedState{
        NOT_GRABBED,
        GRABBED,
        HAS_TO_BE_GRABBED,
        HAS_TO_BE_RELEASED
    };

    const std::string LINK_PREFIX = "grid::link_";
    std::vector<std::vector<geometry_msgs::Pose> > link_poses;
    std::vector<GrabbedState> grabbed;
    std::vector<PSIPtr> planning_scenes;
    std::vector<std::vector<moveit_msgs::CollisionObject> > collision_objects;
    std::vector<std::string> planning_frames;
    std::shared_ptr<std::thread> moveit_object_updater;
    std::vector<const RobotInterface*> robot_interfaces;
    const int num_robots;
    int hres, vres;
    double size[2], offset[3];
    float sphere_radius, sphere_safe_threshold;
    bool up = true;
    bool ready = false;

    void updateMoveItObjects();
    void applyCollisionObjects();
public:
    GridState(std::vector<double> size, std::vector<int> resolution, std::vector<double> offset, float sphere_radius, std::vector<PSIPtr> psi, RobotInterface robots[], int num_robots, float sphere_safe_threshold = 0.025);
    ~GridState();
    void updateCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
    const geometry_msgs::Pose& getPose(int i, int j) const;
    bool isReady() const;
    void shutdown();
    void setGrabbed(int robot_i, bool grab);
};