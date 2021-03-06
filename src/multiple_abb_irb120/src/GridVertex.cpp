#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "GridVertex.hpp"

void GridVertex::setLink(gazebo::physics::LinkPtr link) {
    this->link = link;
}
const gazebo::physics::LinkPtr GridVertex::getLink() {
    return link;
}
ignition::math::Vector3d GridVertex::getForceCache() {
    return force_cache;
}
void GridVertex::setForceCache(ignition::math::Vector3d force_cache) {
    this->force_cache = force_cache;
}
void GridVertex::update(){
    if(grabbed){
        //link->SetPosition();
    }
    else{
        link->SetForce(force_cache);
    }
}
bool GridVertex::isGrabbed() {
    return grabbed;
}
void GridVertex::setGrabbed(bool grabbed) {
    this->grabbed = grabbed;
    link->SetLinkStatic(grabbed);
    link->SetGravityMode(!grabbed);
}
