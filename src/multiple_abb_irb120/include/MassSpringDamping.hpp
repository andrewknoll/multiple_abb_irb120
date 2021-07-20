#pragma once
#include "Grid.hpp"

class MassSpringDamping {
protected:
    double mass, stiffness, damping;
    bool gravity = true;
public:
    MassSpringDamping(double mass, double stiffness, double damping, bool simulateGravity = true);
    virtual void computePositions(std::shared_ptr<multiple_abb_irb120::Grid> grid, double ts);
};