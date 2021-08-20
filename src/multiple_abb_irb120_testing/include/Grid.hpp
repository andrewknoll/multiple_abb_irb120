#pragma once

#include <ros/ros.h>
#include "GazeboSphere.hpp"

float calculateInitialPos(int index, float offset, float size, int resolution);

namespace multiple_abb_irb120{
    
    class Grid{
    private:
        ros::NodeHandle* nh;
        float x, y, z, w, h;
        int hr, vr;
        std::vector<std::vector<GazeboSphere*> > grid;
    public:
        Grid(ros::NodeHandle* nh, float x_origin, float y_origin, float z_origin, float width, float height, int hres, int vres);
        const GazeboSphere& getSphere(int i, int j);
        void update();
    };
}
