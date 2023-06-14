#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <utility>

struct Solution {
    Eigen::VectorXd z;
    int i;
};

Solution empc_solution_pQP1_KinematicsBicycleModelWithDelay_7_0_1(const Eigen::VectorXd& x);
