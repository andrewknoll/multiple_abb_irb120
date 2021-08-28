#include <vector>
#include "geometry_msgs/Pose.h"
#include "gazebo/gazebo.hh"
#include "gazebo_msgs/LinkStates.h"
#include "GridState.hpp"
#include <mutex>
#include <condition_variable>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <thread>
#include "utils.hpp"

#include <ros/console.h>

GridState::GridState(std::vector<double> size,
                     std::vector<int> resolution,
                     std::vector<double> offset,
                     float sphere_radius,
                     std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi, 
                     std::string planning_frame,
                     float sphere_safe_threshold)
: size{size[0], size[1]},
  vres(resolution[0]),
  hres(resolution[1]),
  sphere_radius(sphere_radius),
  offset{offset[0], offset[1], offset[2]},
  planning_scene(psi),
  planning_frame(planning_frame),
  sphere_safe_threshold(sphere_safe_threshold) {
    
    this->link_poses = std::vector<std::vector<geometry_msgs::Pose> >(vres);
    this->collision_objects = std::vector<moveit_msgs::CollisionObject>(vres * hres);
    int indices[3] = {0, 0, 0};
    int resolution_array[3] = {hres, vres, 1};
    grabbed = std::vector<std::vector<GrabbedState> > (vres);
    std::cout << "Starting GridState" << std::endl;
    for(int i = 0; i < vres; i++){
        grabbed[i] = std::vector<GrabbedState>(hres);
        indices[1] = i;
        //Initialize column
        link_poses[i] = std::vector<geometry_msgs::Pose>(hres);
        for(int j = 0; j < hres; j++){
            indices[0] = j;
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = planning_frame;
            collision_object.id = "sphere_zone_" + std::to_string(i * hres + j);
            std::cout << "sphere_zone_" << i*hres + j << std::endl;

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(1);
            primitive.dimensions[0] = sphere_radius + sphere_safe_threshold;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(utils::calculateInitialPos(indices, this->offset, this->size, resolution_array));
            collision_object.operation = collision_object.ADD;

            collision_objects.push_back(collision_object);
            grabbed[i][j] = NOT_GRABBED;
            std::cout << "Size: " << collision_object.primitives.size() << std::endl;
        }
    }
    planning_scene->applyCollisionObjects(collision_objects);
    moveit_object_updater = std::make_shared<std::thread>(&GridState::updateMoveItObjects, this);
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
                ready = true;
                must_update = true;
                cv.notify_all();
            }
            catch(...){}
        }
    }
}

void GridState::updateMoveItObjects(){
    while(up){
        std::unique_lock<std::mutex> lck(mtx);
        cv.wait(lck, [this]{return must_update;});
        while(must_update && ready){
            must_update = false;
            collision_objects.clear();
            for(int i = 0; i < vres; i++){
                for(int j = 0; j < hres; j++){
                    if(grabbed[i][j] == GRABBED) continue;
                    moveit_msgs::CollisionObject collision_object;
                    collision_object.header.frame_id = planning_frame;
                    collision_object.id = "sphere_zone_" + std::to_string(i * hres + j);
                    collision_object.primitive_poses.push_back(this->link_poses[i][j]);

                    if(grabbed[i][j] == HAS_TO_BE_GRABBED){
                        collision_object.operation = collision_object.REMOVE;
                        grabbed[i][j] = GRABBED;
                    }
                    else if(grabbed[i][j] == HAS_TO_BE_RELEASED){
                        shape_msgs::SolidPrimitive primitive;
                        primitive.type = primitive.SPHERE;
                        primitive.dimensions.resize(1);
                        primitive.dimensions[0] = sphere_radius + sphere_safe_threshold;

                        collision_object.primitives.push_back(primitive);
                        collision_object.operation = collision_object.ADD;
                        grabbed[i][j] = NOT_GRABBED;
                    }
                    else {
                        collision_object.operation = collision_object.MOVE;
                    }
                    collision_objects.push_back(collision_object);
                }
            }
            planning_scene->applyCollisionObjects(collision_objects);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

const geometry_msgs::Pose& GridState::getPose(int i, int j) const{
    return this->link_poses[i][j];
}

bool GridState::isReady() const {
    return ready;
}

void GridState::shutdown(){
    if(up){
        up = false;
        must_update = false;
        ready = false;
        cv.notify_all();
        moveit_object_updater->join();
    }
}

GridState::~GridState() {
    shutdown();
}

void GridState::setGrabbed(int i, int j, bool grab){
    if(grabbed[i][j] != GRABBED && grab){
        grabbed[i][j] = HAS_TO_BE_GRABBED;
        must_update = true;
        cv.notify_all();
    }
    else if(grabbed[i][j] != NOT_GRABBED && !grab){
        grabbed[i][j] = HAS_TO_BE_RELEASED;
        must_update = true;
        cv.notify_all();
    }
}