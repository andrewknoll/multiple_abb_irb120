#include <string>
#include <vector>
#include <ros/ros.h>
#include "GazeboSphere.hpp"
#include "Grid.hpp"

using namespace multiple_abb_irb120;

float calculateInitialPos(int index, float offset, float size, int resolution) {
    return offset - (size / 2) + (float)index * (size / (float)resolution);
}

Grid::Grid(ros::NodeHandle* nh, float x_origin, float y_origin, float width, float height, int hres, int vres) :
    nh(nh), x(x_origin), y(y_origin), w(width), h(height), hr(hres), vr(vres)
{
    grid = std::vector <std::vector<GazeboSphere*> >(vres);
    for(int i = 0; i < vres; i++) {
        grid[i] = std::vector<GazeboSphere*>(hres);
        for(int j = 0; j < hres; j++) {
            grid[i][j] = new GazeboSphere(nh);
            grid[i][j]->setPosition(calculateInitialPos(i, x_origin, width, hres), calculateInitialPos(j, y_origin, height, vres), 1);
        }
    }
}

const GazeboSphere& Grid::getSphere(int i, int j) {
    return *grid[i][j];
}

void Grid::update(){
    for(int i = 0; i < hr; i++) {
        for(int j = 0; j < vr; j++) {
            grid[i][j]->update();
        }
    }
}
