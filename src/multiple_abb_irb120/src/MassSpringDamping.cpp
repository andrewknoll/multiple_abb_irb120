#include "MassSpringDamping.hpp"
#include "Grid.hpp"
#include <Eigen/Core>
#include "utils.hpp"



MassSpringDamping::MassSpringDamping(double mass, double stiffness, double damping, bool simulateGravity) :
    mass(mass), stiffness(stiffness), damping(damping), gravity(simulateGravity) {}

void MassSpringDamping::computePositions(std::shared_ptr<multiple_abb_irb120::Grid> grid, double ts) {
    int c;
    double n, n0;
    double ts_p2 = ts * ts;
    Eigen::Matrix<double, 1, 8> ones = Eigen::Matrix<double, 1, 8>::Ones();
    Eigen::Matrix<double, 1, 8> k = stiffness * ones;
    Eigen::Matrix<double, 8, 3> x = Eigen::Matrix<double, 8, 3>::Zero();
    Eigen::Vector3d f_g, forces;
    ignition::math::Vector3d result;
    f_g << 0, 0, -9.81;

    ignition::math::Vector3d l0, lt;
    
    for(int i = 0; i < grid->getRows(); i++) {
        for(int j = 0; j < grid->getColumns(); j++) {
            for(int neighbour_i = -1; neighbour_i <= 1; neighbour_i++) {
                if(neighbour_i + i < 0
                || neighbour_i + i >= grid->getRows())
                continue;
                for(int neighbour_j = -1; neighbour_j <= 1; neighbour_j++) {
                    if(neighbour_j == 0 && neighbour_i == 0
                    || neighbour_j + j < 0
                    || neighbour_j + j >= grid->getColumns())
                    continue;
                    c = neighbour_j + 1 + 3*(neighbour_i + 1);
                    if(c > 3) c--;
                    l0 = grid->get(i, j)->InitialRelativePose().Pos() - grid->get(i + neighbour_i, j + neighbour_j)->InitialRelativePose().Pos();
                    lt = grid->get(i, j)->RelativePose().Pos() - grid->get(i + neighbour_i, j + neighbour_j)->RelativePose().Pos();
                    n0 = normalize(l0);
                    n = normalize(lt);
                    x.row(c) = toEigenVector3d((n0 - n) * (lt / n));
                }
            }
            forces = k*x;
            //Forces manually applied + damping + gravity
            result = toIgnitionVector3d(forces) + damping * grid->get(i, j)->RelativeLinearVel();
            if(gravity){
                result += toIgnitionVector3d(mass * f_g);
            }
            grid->get(i, j)->SetForce(result);
        }
    }
    /*
    //ts is assumed to be small
    for(int i = 0; i < grid.getRows(); i++){
        for(int j = 0; j < grid.getColumns(); j++){
            grid(i, j).setAcc(grid(i, j).getF() / mass);
            //cmd_vel ????
            grid(i, j).setVel(grid(i, j).getVel() + grid(i, j).getAcc() * ts);
            grid(i, j).setPosition(grid(i, j)->RelativePose() + toPoint(grid(i, j).getVel() * ts));
        }
    }*/
}
