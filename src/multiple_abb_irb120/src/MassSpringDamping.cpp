#include "MassSpringDamping.hpp"
#include "Grid.hpp"
#include <Eigen/Core>
#include "utils.hpp"

MassSpringDamping::MassSpringDamping(float mass, float stiffness, float damping, bool simulateGravity) :
    mass(mass), stiffness(stiffness), damping(damping), gravity(simulateGravity) {}

void MassSpringDamping::computePositions(multiple_abb_irb120::Grid grid, float ts) {
    int c;
    float n, n0;
    float ts_p2 = ts * ts;
    Eigen::Matrix<float, 1, 8> ones = Eigen::Matrix<float, 1, 8>::Ones();
    Eigen::Matrix<float, 1, 8> k = stiffness * ones;
    Eigen::Matrix<float, 8, 3> x = Eigen::Matrix<float, 8, 3>::Zero();
    Eigen::Vector3f f_g, forces, result;
    f_g << 0, 0, -9.81;

    geometry_msgs::Point l0, lt;
    
    for(int i = 0; i < grid.getRows(); i++) {
        for(int j = 0; j < grid.getColumns(); j++) {
            for(int neighbour_i = -1; neighbour_i <= 1; neighbour_i++) {
                if(neighbour_i + i < 0
                || neighbour_i + i >= grid.getRows())
                continue;
                for(int neighbour_j = -1; neighbour_j <= 1; neighbour_j++) {
                    if(neighbour_j == 0 && neighbour_i == 0
                    || neighbour_j + j < 0
                    || neighbour_j + j >= grid.getColumns())
                    continue;
                    c = neighbour_j + 1 + 3*(neighbour_i + 1);
                    if(c > 3) c--;
                    l0 = grid(i, j).getInitialPos() - grid(i + neighbour_i, j + neighbour_j).getInitialPos();
                    lt = grid(i, j).getCurrentPos() - grid(i + neighbour_i, j + neighbour_j).getCurrentPos();
                    n0 = normalize(l0);
                    n = normalize(lt);
                    x.row(c) = toRow((n0 - n) * (lt / n));
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
    
    //ts is assumed to be small
    for(int i = 0; i < grid.getRows(); i++){
        for(int j = 0; j < grid.getColumns(); j++){
            grid(i, j).setAcc(grid(i, j).getF() / mass);
            //cmd_vel ????
            grid(i, j).setVel(grid(i, j).getVel() + grid(i, j).getAcc() * ts);
            grid(i, j).setPosition(grid(i, j).getCurrentPos() + toPoint(grid(i, j).getVel() * ts));
        }
    }
}
