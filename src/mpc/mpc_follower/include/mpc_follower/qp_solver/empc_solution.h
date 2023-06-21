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

Solution empc_solution_pQP1_ff_KinematicsBicycleModelWithDelay_7_0_1(const Eigen::VectorXd& x,
                                                                   const Eigen::MatrixXd& H,
                                                                   const std::vector<int>& ni,
                                                                   const Eigen::MatrixXd& fF,
                                                                   const Eigen::MatrixXd& tF,
                                                                   const std::vector<double>& tg,
                                                                   const Eigen::MatrixXd& tH,
                                                                   const Eigen::MatrixXd& fg);
