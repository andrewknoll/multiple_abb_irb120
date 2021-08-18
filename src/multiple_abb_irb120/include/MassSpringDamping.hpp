#pragma once
#include "Grid.hpp"
#include <ignition/math/Pose3.hh>

class MassSpringDamping {
protected:
    double mass, stiffness, damping;
    ignition::math::Vector3d g;
    bool gravity = true;
    const ignition::math::Vector3d f_g = {0, 0, -9.81};
    void computeRegion(std::shared_ptr<multiple_abb_irb120::Grid> grid/*, double ts*/, int regionPos, int regionSize);
public:
    MassSpringDamping(double mass, double stiffness, double damping, bool simulateGravity = true, ignition::math::Vector3d gravity = {0, 0, -9.81});
    virtual void computePositions(std::shared_ptr<multiple_abb_irb120::Grid> grid/*, double ts*/);
};