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
#include "RobotInterface.hpp"

#include <ros/console.h>

template <typename T>
std::vector<const T*> createFromArray(const T* array, size_t size) {
    std::vector<const T*> vec;
    for(size_t i = 0; i < size; ++i) {
        vec.push_back(&array[i]);
    }
    std::cout << "tuto bene 2" << std::endl;
    return vec;
}

GridState::GridState(std::vector<double> size,
                     std::vector<int> resolution,
                     std::vector<double> offset,
                     float sphere_radius,
                     std::vector<GridState::PSIPtr> psi, 
                     RobotInterface robots[],
                     int num_robots,
                     float sphere_safe_threshold)
: size{size[0], size[1]},
  vres(resolution[0]),
  hres(resolution[1]),
  sphere_radius(sphere_radius),
  offset{offset[0], offset[1], offset[2]},
  planning_scenes(psi),
  num_robots(num_robots),
  robot_interfaces(createFromArray(robots, num_robots)),
  sphere_safe_threshold(sphere_safe_threshold) {
    this->link_poses = std::vector<std::vector<geometry_msgs::Pose> >(vres);
    this->collision_objects = std::vector<std::vector<moveit_msgs::CollisionObject> >(num_robots);
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
            for(int r = 0; r < num_robots; ++r) {
                moveit_msgs::CollisionObject collision_object;
                
                
                collision_object.id = "sphere_zone_" + std::to_string(i * hres + j);
                collision_object.header.frame_id = "world";
                //std::cout << "sphere_zone_" << i*hres + j << std::endl;

                shape_msgs::SolidPrimitive primitive;
                primitive.type = primitive.SPHERE;
                primitive.dimensions.resize(1);
                primitive.dimensions[0] = sphere_radius + sphere_safe_threshold;

                collision_object.primitives.push_back(primitive);
                geometry_msgs::Pose pose = utils::calculateInitialPos(indices, this->offset, this->size, resolution_array);
                geometry_msgs::Point base_position = robot_interfaces[r]->getBasePosition();
                pose.position.x -= base_position.x;
                pose.position.y -= base_position.y;
                pose.position.z -= base_position.z;
                collision_object.primitive_poses.push_back(pose);
                collision_object.operation = collision_object.ADD;

                collision_objects[r].push_back(collision_object);
                
            }
            grabbed[i][j] = NOT_GRABBED;
        }
    }
    applyCollisionObjects();
    
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
            for(int r = 0; r < num_robots; ++r) {
                collision_objects[r].clear();
            }
            for(int i = 0; i < vres; i++){
                for(int j = 0; j < hres; j++){
                    if(grabbed[i][j] == GRABBED) continue;
                    for(int r = 0; r < num_robots; ++r) {
                        moveit_msgs::CollisionObject collision_object;
                        collision_object.id = "sphere_zone_" + std::to_string(i * hres + j);
                        collision_object.header.frame_id = "world";
                        geometry_msgs::Pose pose = link_poses[i][j];
                        geometry_msgs::Point base_position = robot_interfaces[r]->getBasePosition();
                        pose.position.x -= base_position.x;
                        pose.position.y -= base_position.y;
                        pose.position.z -= base_position.z;

                        collision_object.primitive_poses.push_back(pose);

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
                        collision_objects[r].push_back(collision_object);
                    }
                }
            }
            applyCollisionObjects();
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

void GridState::applyCollisionObjects(){
    for(int i = 0; i < planning_scenes.size(); i++){
        planning_scenes[i]->applyCollisionObjects(collision_objects[i]);
    }
}