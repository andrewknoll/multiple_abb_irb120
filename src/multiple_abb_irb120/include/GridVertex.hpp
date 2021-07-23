#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

class GridVertex {
private:
    gazebo::physics::LinkPtr link;
    ignition::math::Vector3d force_cache;
    bool grabbed = false;
public:
    void setLink(gazebo::physics::LinkPtr link);
    const gazebo::physics::LinkPtr getLink();
    void setForceCache(ignition::math::Vector3d force_cache);
    void update();

    bool isGrabbed();
    void setGrabbed(bool grabbed);
};