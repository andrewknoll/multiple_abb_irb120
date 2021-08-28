#include <vector>
#include <geometry_msgs/Pose.h>
#include "gazebo/gazebo.hh"
#include <gazebo_msgs/LinkStates.h>
#include <mutex>
#include <condition_variable>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <thread>

class GridState {
private:
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
    std::vector<std::vector<GrabbedState> > grabbed;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::string planning_frame;
    std::shared_ptr<std::thread> moveit_object_updater;
    int hres, vres;
    double size[2], offset[3];
    float sphere_radius, sphere_safe_threshold;
    bool up = true;
    bool ready = false;

    void updateMoveItObjects();
public:
    GridState(std::vector<double> size, std::vector<int> resolution, std::vector<double> offset, float sphere_radius, std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi, std::string planning_frame, float sphere_safe_threshold = 0.025);
    ~GridState();
    void updateCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
    const geometry_msgs::Pose& getPose(int i, int j) const;
    bool isReady() const;
    void shutdown();
    void setGrabbed(int i, int j, bool grab);
};