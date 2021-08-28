#pragma once

#include "GridVertex.hpp"
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace multiple_abb_irb120{
    
    class Grid{
    private:
        double x, y, z, w, h;
        int hr, vr;
        gazebo::physics::ModelPtr model;
        std::vector<std::vector<GridVertex> > grid;
        bool testing = false;
    public:
        Grid(gazebo::physics::ModelPtr model, double x_origin, double y_origin, double z_origin, double width, double height, int hres, int vres, bool testing = false);
        GridVertex& get(int i, int j);
        void setLink(int i, int j, gazebo::physics::LinkPtr link);
        const gazebo::physics::LinkPtr getLink(int i, int j);
        int getRows();
        int getColumns();
        void update();
    };
}
