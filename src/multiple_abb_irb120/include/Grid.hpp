#pragma once

#include "GridVertex.hpp"
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

double calculateInitialComponent(int index, double offset, double size, int resolution);

gazebo::msgs::Pose* calculateInitialPos(int indices[3], double offset[3], double size[3], int resolution[3]);

namespace multiple_abb_irb120{
    
    class Grid{
    private:
        double x, y, z, w, h;
        int hr, vr;
        gazebo::physics::ModelPtr model;
        std::vector<std::vector<GridVertex> > grid;
    public:
        Grid(gazebo::physics::ModelPtr model, double x_origin, double y_origin, double z_origin, double width, double height, int hres, int vres);
        GridVertex& get(int i, int j);
        void setLink(int i, int j, gazebo::physics::LinkPtr link);
        const gazebo::physics::LinkPtr getLink(int i, int j);
        int getRows();
        int getColumns();
        void update();
    };
}
