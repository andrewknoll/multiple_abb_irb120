#include "MassSpringDamping.hpp"
#include "Grid.hpp"
#include <Eigen/Core>
#include "utils.hpp"

MassSpringDamping::MassSpringDamping(float mass, float stiffness, float damping, bool simulateGravity) :
    mass(mass), stiffness(stiffness), damping(damping), gravity(simulateGravity) {}

void MassSpringDamping::computePositions(multiple_abb_irb120::Grid grid, float ts) {
    int c;
    float n;
    float ts_p2 = ts * ts;
    Eigen::Matrix<float, 1, 8> ones = Eigen::Matrix<float, 1, 8>::Ones();
    Eigen::Matrix<float, 1, 8> k = stiffness * ones;
    Eigen::Matrix<float, 8, 3> x = Eigen::Matrix<float, 8, 3>::Zero();
    Eigen::Vector3f f_g, forces, result;
    f_g << 0, 0, -9.81;

    geometry_msgs::Point l0, lt;
    
    for(int i = 0; i < grid.getRows(); i++) {
        for(int j = 0; j < grid.getColumns(); j++) {
            for(int neighbour_y = -1; neighbour_y <= 1; neighbour_y++) {
                if(neighbour_y + i < 0
                || neighbour_y + i >= grid.getRows())
                continue;
                for(int neighbour_x = -1; neighbour_x <= 1; neighbour_x++) {
                    if(neighbour_x == 0 && neighbour_y == 0
                    || neighbour_x + j < 0
                    || neighbour_x + j >= grid.getColumns())
                    continue;
                    c = neighbour_x + 1 + 3*(neighbour_y + 1);
                    if(c > 3) c--;
                    l0 = grid(i, j).getInitialPos() - grid(i + neighbour_y, j + neighbour_x).getInitialPos();
                    lt = grid(i, j).getCurrentPos() - grid(i + neighbour_y, j + neighbour_x).getCurrentPos();
                    n = normalize(lt);
                    x.row(c) = toRow((normalize(l0) - n) * lt / n);
                }
            }
            forces = k*x;
            //Forces manually applied + damping + gravity
            result = forces + damping * grid(i, j).getVel();
            if(gravity){
                result += mass * f_g;
            }
            grid(i, j).setF(result);
        }
    }

    for(int i = 0; i < grid.getRows(); i++){
        for(int j = 0; j < grid.getColumns(); j++){
            grid(i, j).setAcc(grid(i, j).getF() / mass);
            grid(i, j).setVel(grid(i, j).getVel() + grid(i, j).getAcc() * ts);
            grid(i, j).setPosition(grid(i, j).getCurrentPos() + toPoint(grid(i, j).getVel() * ts));
        }
    }
}
