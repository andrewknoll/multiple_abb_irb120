#include "MassSpringDamping.hpp"
#include "Grid.hpp"
#include <Eigen/Core>
#include "utils.hpp"
#include <numeric>


MassSpringDamping::MassSpringDamping(double mass, double stiffness, double damping, bool simulateGravity) :
    mass(mass), stiffness(stiffness), damping(damping), gravity(simulateGravity) {}

void MassSpringDamping::computePositions(std::shared_ptr<multiple_abb_irb120::Grid> grid, double ts) {
    int c;
    double n, n0;
    double ts_p2 = ts * ts;
    std::vector <ignition::math::Vector3d> x(8);
    ignition::math::Vector3d forces, f_g = {0, 0, -9.81};
    ignition::math::Vector3d result;

    ignition::math::Vector3d l0, lt;
    
    for(int i = 0; i < grid->getRows(); i++) {
        for(int j = 0; j < grid->getColumns(); j++) {
            if(grid->get(i, j).isGrabbed()) continue;
            for(int neighbour_i = -1; neighbour_i <= 1; neighbour_i++) {
                if(neighbour_i + i < 0
                || neighbour_i + i >= grid->getRows())
                continue;
                for(int neighbour_j = -1; neighbour_j <= 1; neighbour_j++) {
                    if((neighbour_j == 0 && neighbour_i == 0)
                    || neighbour_j + j < 0
                    || neighbour_j + j >= grid->getColumns())
                    continue;
                    c = neighbour_j + 1 + 3*(neighbour_i + 1);
                    if(c > 3) c--;
                    //Initial pos is relative to grid
                    l0 = grid->getLink(i, j)->InitialRelativePose().Pos() - grid->getLink(i + neighbour_i, j + neighbour_j)->InitialRelativePose().Pos();
                    //Current pos is in world coordinates (since it's a substraction, the result is also relative)
                    lt = grid->getLink(i, j)->WorldCoGPose().Pos() - grid->getLink(i + neighbour_i, j + neighbour_j)->WorldCoGPose().Pos();
                    n0 = l0.Length();
                    n = lt.Length();
                    x[c] = stiffness * (n0 - n) * (lt / n);
                    if(i == 1 && j == 1) {
                        std::cout << "n: " << n << std::endl;
                        std::cout << "neigh: " << neighbour_i << " " << neighbour_j << std::endl;
                        std::cout << "inipos: " << grid->getLink(i + neighbour_i, j + neighbour_j)->InitialRelativePose().Pos() << std::endl;
                        std::cout << "pos: " << grid->getLink(i + neighbour_i, j + neighbour_j)->WorldCoGPose().Pos() << std::endl;
                        std::cout << "row: " << x[c].X() << " " << x[c].Y() << " " << x[c].Z() << std::endl;
                    }
                }
            }
            forces = std::accumulate(x.begin(), x.end(), ignition::math::Vector3d::Zero);
            //Forces manually applied - damping + gravity
            result = forces - damping * grid->getLink(i, j)->RelativeLinearVel();
            if(i == 1 && j == 1) {
                        std::cout << "vel: " << grid->getLink(i, j)->RelativeLinearVel().X() << " " << grid->getLink(i, j)->RelativeLinearVel().Y() << " " << grid->getLink(i, j)->RelativeLinearVel().Z() << std::endl;
                    }
            if(gravity){
                result += mass * f_g;
            }
            grid->get(i, j).setForceCache(result);
            if(i == 1 && j == 1) {
                std::cout << "Forces: " << result.X() << " " << result.Y() << " " << result.Z() << std::endl;
                std::cout << "Applied: " << grid->getLink(i, j)->RelativeForce().X() << " " << grid->getLink(i, j)->RelativeForce().Y() << " " << grid->getLink(i, j)->RelativeForce().Z() << std::endl;
            }
        }
    }
    
    //ts is assumed to be small
    /*for(int i = 0; i < grid->getRows(); i++){
        for(int j = 0; j < grid->getColumns(); j++){
            //grid->getLink(i, j).setAcc(grid(i, j).getF() / mass);
            grid->getLink(i, j)->SetLinearVel(grid->getLink(i, j)->RelativeLinearVel() + grid->get(i, j).getForceCache() / mass * ts);
            //grid(i, j).setPosition(grid(i, j)->RelativePose() + toPoint(grid(i, j).getVel() * ts));
        }
    }*/
}
