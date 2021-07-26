#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

class GridVertex {
private:
    gazebo::physics::LinkPtr link;
    ignition::math::Vector3d force_cache;
    ignition::math::Vector3d initial_pos;
    bool grabbed = false;
public:
    void setLink(gazebo::physics::LinkPtr link);
    const gazebo::physics::LinkPtr getLink();
    ignition::math::Vector3d getForceCache();
    void setForceCache(ignition::math::Vector3d force_cache);
    ignition::math::Vector3d getInitialPos();
    void update();

    bool isGrabbed();
    void setGrabbed(bool grabbed);
};