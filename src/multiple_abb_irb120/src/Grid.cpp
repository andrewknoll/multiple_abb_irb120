#include <string>
#include <vector>
#include <ros/ros.h>
#include "Grid.hpp"
#include "utils.hpp"

using namespace multiple_abb_irb120;

const double SPHERE_MASS = 0.1;
const double SPHERE_RADIUS = 0.025;

double calculateInitialComponent(int index, double offset, double size, int resolution) {
    return offset - (size / 2) + (double)index * (size / (double)resolution);
}

gazebo::msgs::Pose* calculateInitialPos(int indices[3], double offset[3], double size[3], int resolution[3]){
    gazebo::msgs::Pose* pos = new gazebo::msgs::Pose();
    gazebo::msgs::Vector3d* position = new gazebo::msgs::Vector3d();
    position->set_x(calculateInitialComponent(indices[0], offset[0], size[0], resolution[0]));
    position->set_y(calculateInitialComponent(indices[1], offset[1], size[1], resolution[1]));
    position->set_z(calculateInitialComponent(indices[2], offset[2], size[2], resolution[2]));
    gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
    pos->set_allocated_position(position);
    pos->set_allocated_orientation(orientation);

    return pos;
}

Grid::Grid(gazebo::physics::ModelPtr model, double x_origin, double y_origin, double z_origin, double width, double height, int hres, int vres) :
    model(model), x(x_origin), y(y_origin), z(z_origin), w(width), h(height), hr(hres), vr(vres)
{
    double offset[3] = {x_origin, y_origin, z_origin};
    double size[3] = {width, height, 1};
    int resolution[3] = {hres, vres, 1};
    int indices[3] = {0, 0, 0};
    gazebo::msgs::Pose* pos;
    std::string linkName;
    grid = std::vector <std::vector<gazebo::physics::LinkPtr > >(vres);
    for(int i = 0; i < vres; i++) {
        indices[0] = i;
        grid[i] = std::vector<gazebo::physics::LinkPtr >(hres);
        for(int j = 0; j < hres; j++) {
            indices[1] = j;
            pos = calculateInitialPos(indices, offset, size, resolution);
            linkName = "link_" + std::to_string(i * width + j);
            makeLinkWithSphereShape(this->model, linkName, SPHERE_MASS, SPHERE_RADIUS, pos);
            grid[i][j] = this->model->GetLink(linkName);
            if(grid[i][j] == NULL) {
                std::cout << this->model->GetLinks().size() << std::endl;
            }
            else{
                std::cout << grid[i][j]->GetSDF()->ToString("") << std::endl;
            }
            
        }
    }
}

gazebo::physics::LinkPtr Grid::get(int i, int j) {
    return grid[i][j];
}

void Grid::setLink(int i, int j, gazebo::physics::LinkPtr link) {
    grid[i][j] = link;
}

int Grid::getRows(){
    return vr;
}

int Grid::getColumns(){
    return hr;
}