#include <string>
#include <vector>
#include <ros/ros.h>
#include "Grid.hpp"
#include "utils.hpp"
#include "GridVertex.hpp"

using namespace multiple_abb_irb120;

Grid::Grid(gazebo::physics::ModelPtr model, double x_origin, double y_origin, double z_origin, double width, double height, int hres, int vres) :
    model(model), x(x_origin), y(y_origin), z(z_origin), w(width), h(height), hr(hres), vr(vres)
{
    //Initialize rows
    grid = std::vector <std::vector<GridVertex> >(vres);

    for(int i = 0; i < vr; i++){
        //Initialize column
        grid[i] = std::vector<GridVertex>(hres);
        for(int j = 0; j < hr; j++){
            grid[i][j].setLink(model->GetLink("link_" + std::to_string(i * (int)width + j)));
        }
    }
}

GridVertex& Grid::get(int i, int j) {
    return grid[i][j];
}

void Grid::setLink(int i, int j, gazebo::physics::LinkPtr link) {
    grid[i][j].setLink(link);
}

const gazebo::physics::LinkPtr Grid::getLink(int i, int j) {
    return grid[i][j].getLink();
}

int Grid::getRows(){
    return vr;
}

int Grid::getColumns(){
    return hr;
}

void Grid::update() {
    for(int i = 0; i < vr; i++){
        for(int j = 0; j < hr; j++){
            grid[i][j].update();
        }
    }
}