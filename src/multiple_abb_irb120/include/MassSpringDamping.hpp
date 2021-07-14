#include "Grid.hpp"

class MassSpringDamping {
protected:
    float mass, stiffness, damping;
    bool gravity = true;
public:
    MassSpringDamping(float mass, float stiffness, float damping, bool simulateGravity = true);
    virtual void computePositions(multiple_abb_irb120::Grid grid, float ts);
};