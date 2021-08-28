#include "MassSpringDamping.hpp"
#include "Grid.hpp"
#include <Eigen/Core>
#include "utils.hpp"
#include <numeric>
#include <thread>
#include <ignition/math/Pose3.hh>


MassSpringDamping::MassSpringDamping(double mass, double stiffness, double damping, bool simulateGravity, ignition::math::Vector3d gravity) :
    mass(mass), stiffness(stiffness), damping(damping), gravity(simulateGravity), g(gravity) {}

void MassSpringDamping::computePositions(std::shared_ptr<multiple_abb_irb120::Grid> grid/*, double ts*/) {
    
    unsigned int nThreads = 1/*std::thread::hardware_concurrency()*/;

    std::vector<std::thread> threads_vec;

    int s = (grid->getRows() * grid->getColumns());
    int n = s / nThreads;

    for(int t = 0; t < nThreads - 1; t++) {
        threads_vec.emplace_back(&MassSpringDamping::computeRegion, this, grid, t*n, s);
    }
    computeRegion(grid, 0, n);

    for(int i = 0; i < nThreads - 1; i++){
        threads_vec[i].join();
    }
}

void MassSpringDamping::computeRegion(std::shared_ptr<multiple_abb_irb120::Grid> grid/*, double ts*/, int regionPos, int regionSize) {
    int i, j;
    double n, n0;
    //double ts_p2 = ts * ts;
    std::vector <ignition::math::Vector3d> f, d;
    ignition::math::Vector3d forces, damping_forces;
    ignition::math::Vector3d result;

    ignition::math::Vector3d l0, lt;

    while(regionPos < regionSize){
        i = regionPos / grid->getColumns();
        j = regionPos % grid->getColumns();
        

        if(grid->get(i, j).isGrabbed()){
            f.clear();
            d.clear();
            regionPos++;
            continue;
        }
        for(int neighbour_i = -1; neighbour_i <= 1; neighbour_i++) {
            if(neighbour_i + i < 0
            || neighbour_i + i >= grid->getRows())
            continue;
            for(int neighbour_j = -1; neighbour_j <= 1; neighbour_j++) {
                if((neighbour_j == 0 && neighbour_i == 0)
                || neighbour_j + j < 0
                || neighbour_j + j >= grid->getColumns())
                continue;
                //Initial pos is relative to grid
                l0 = grid->getLink(i, j)->InitialRelativePose().Pos() - grid->getLink(i + neighbour_i, j + neighbour_j)->InitialRelativePose().Pos();
                lt = grid->getLink(i, j)->WorldPose().Pos() - grid->getLink(i + neighbour_i, j + neighbour_j)->WorldPose().Pos();
                n0 = l0.Length();
                n = lt.Length();
                f.push_back(-stiffness * (n - n0) * (lt / n));
                d.push_back(-damping * (grid->getLink(i, j)->WorldLinearVel() - grid->getLink(i + neighbour_i, j + neighbour_j)->WorldLinearVel()));
                if(i == 1 && j == 1) {
                    std::cout << "=================================" << std::endl;
                    std::cout << "neigh: " << neighbour_i << " " << neighbour_j << std::endl;
                    std::cout << "n: " << n << std::endl;
                    std::cout << "n0:" << n0 << std::endl;
                    std::cout << "l0: " << l0.X() << " " << l0.Y() << " " << l0.Z() << std::endl;
                    std::cout << "lt: " << lt.X() << " " << lt.Y() << " " << lt.Z() << std::endl;
                    std::cout << "myInipos: " << grid->getLink(i, j)->InitialRelativePose().Pos() << std::endl;
                    std::cout << "mypos: " << grid->getLink(i, j)->RelativePose().Pos() << std::endl;
                    std::cout << "inipos: " << grid->getLink(i + neighbour_i, j + neighbour_j)->InitialRelativePose().Pos() << std::endl;
                    std::cout << "pos: " << grid->getLink(i + neighbour_i, j + neighbour_j)->WorldPose().Pos() << std::endl;
                    std::cout << "myvel: " << grid->getLink(i, j)->WorldLinearVel() << std::endl;
                    std::cout << "vel: " << grid->getLink(i + neighbour_i, j + neighbour_j)->WorldLinearVel() << std::endl;
                    std::cout << "stiffness: " << f.back().X() << " " << f.back().Y() << " " << f.back().Z() << std::endl;
                    std::cout << "damping: " << d.back().X() << " " << d.back().Y() << " " << d.back().Z() << std::endl;
                }
            }
        }
        forces = std::accumulate(f.begin(), f.end(), ignition::math::Vector3d::Zero);
        damping_forces = std::accumulate(d.begin(), d.end(), ignition::math::Vector3d::Zero);
        //forces = std::accumulate(f.begin(), f.end(), ignition::math::Vector3d::Zero, std::plus<ignition::math::Vector3d>());
        //forces = ignition::math::Vector3d::Zero;
        //Forces manually applied - damping + gravity
        result = forces + damping_forces;
        if(i == 1 && j == 1) {
            std::cout << "vel: " << grid->getLink(i, j)->RelativeLinearVel().X() << " " << grid->getLink(i, j)->RelativeLinearVel().Y() << " " << grid->getLink(i, j)->RelativeLinearVel().Z() << std::endl;
        }
        if(!gravity){
            result -= mass * g;
        }
        grid->get(i, j).setForceCache(result);
        if(i == 1 && j == 1) {
            std::cout << "Forces: " << result.X() << " " << result.Y() << " " << result.Z() << std::endl;
            //std::cout << "Applied: " << grid->getLink(i, j)->RelativeForce().X() << " " << grid->getLink(i, j)->RelativeForce().Y() << " " << grid->getLink(i, j)->RelativeForce().Z() << std::endl;
        }
        f.clear();
        d.clear();
        regionPos++;
    }
    
    //ts is assumed to be small
    /*for(int i = 0; i < grid->getRows(); i++){
        for(int j = 0; j < grid->getColumns(); j++){
            std::cout << "============================================" << std::endl;
            auto asdf = (grid->get(i, j).getForceCache() / mass * ts_p2);
            std::cout << asdf.X() <<" " << asdf.Y() << " " << asdf.Z() << std::endl;
            //grid->getLink(i, j).setAcc(grid(i, j).getF() / mass);
            //grid->getLink(i, j)->SetLinearVel(grid->getLink(i, j)->RelativeLinearVel() + grid->get(i, j).getForceCache() / mass * ts);
            ignition::math::Pose3d p = grid->getLink(i, j)->RelativePose();
            p.CoordPositionAdd(grid->get(i, j).getForceCache() / mass * ts_p2);
            grid->getLink(i, j)->SetRelativePose(p, true, true);
        }
    }*/

    
}
