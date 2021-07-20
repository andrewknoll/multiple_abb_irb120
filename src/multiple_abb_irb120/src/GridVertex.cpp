#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "GridVertex.hpp"

void GridVertex::setLink(gazebo::physics::LinkPtr link) {
    this->link = link;
}
const gazebo::physics::LinkPtr GridVertex::getLink() {
    return link;
}
void GridVertex::setForceCache(ignition::math::Vector3d force_cache) {
    this->force_cache = force_cache;
}
void GridVertex::update(){
    link->SetForce(force_cache);
}
