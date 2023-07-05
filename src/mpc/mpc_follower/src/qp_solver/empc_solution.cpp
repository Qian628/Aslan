#include "mpc_follower/qp_solver/empc_solution.h"

Solution empc_solution_pQP1_ff_KinematicsBicycleModelWithDelay(const Eigen::VectorXd& x,
                                                                   const Eigen::MatrixXd& H,
                                                                   const std::vector<int>& ni,
                                                                   const Eigen::MatrixXd& fF,
                                                                   const Eigen::MatrixXd& tF,
                                                                   const std::vector<double>& tg,
                                                                   const Eigen::MatrixXd& tH,
                                                                   const Eigen::MatrixXd& fg,
                                                                   const int nz,
                                                                   const int nr) 
{
    if (x.size() != 3) {
        throw std::invalid_argument("The input vector must have 3 elements.");
    }

    Eigen::VectorXd xh = Eigen::VectorXd::Zero(4);
    xh << x, -1;
    int nx = 3;

    std::vector<std::pair<int, double>> tb;

    for (int i = 0; i < nr; ++i) 
    {
        if ((H.block(ni[i] - 1, 0, ni[i+1] - ni[i], H.cols()) * xh).maxCoeff() <= 1e-8) 
        {
            double tv = tF.row(i) * x + tg[i];
            if(i>0)
                tv += x.transpose() * tH.block((i-1) * nx, 0, nx, tH.cols()) * x;
            tb.push_back(std::make_pair(i, tv));
        }
    }

    Solution solution;
    solution.z = Eigen::VectorXd::Zero(nz);

    if (!tb.empty()) 
    {
        auto min_pair = *std::min_element(tb.begin(), tb.end(),
            [](const std::pair<int, double>& a, const std::pair<int, double>& b) 
            {
                return a.second < b.second;
            });

        solution.i = min_pair.first;
        solution.z = fF.block((solution.i) * nz, 0, nz, fF.cols()) * x + fg.block((solution.i) * nz, 0, nz, fg.cols());
    } else {
        solution.i = 0;
        solution.z = Eigen::VectorXd::Constant(nz, std::nan(""));
    }

    return solution;
}
