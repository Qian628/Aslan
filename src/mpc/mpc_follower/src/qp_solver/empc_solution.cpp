#include "mpc_follower/qp_solver/empc_solution.h"



Solution empc_solution_pQP1_KinematicsBicycleModelWithDelay_7_0_1(const Eigen::VectorXd& x) 
{
    if (x.size() != 3) {
        throw std::invalid_argument("The input vector must have 3 elements.");
    }

    Eigen::VectorXd xh = Eigen::VectorXd::Zero(4);
    xh << x, -1;

    int nx = 3;
    int nz = 10;

    // Load data from the "empc_data.csv" file
    std::ifstream file("empc_data.csv");
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open the empc_data.csv file.");
    }

    Eigen::MatrixXd H;
    std::vector<int> ni;
    Eigen::MatrixXd fF;
    Eigen::MatrixXd tF;
    std::vector<double> tg;
    Eigen::MatrixXd tH;
    Eigen::MatrixXd fg;

    std::string line;
    int row = 0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (std::getline(ss, cell, ',')) {
            double value = std::stod(cell);
            // Assign the value to the corresponding matrix or vector
            if (row < H.rows()) {
                H(row, col) = value;
            } else if (row < H.rows() + ni.size()) {
                ni[row - H.rows()] = static_cast<int>(value);
            } else if (row < H.rows() + ni.size() + fF.rows()) {
                fF(row - H.rows() - ni.size(), col) = value;
            } else if (row < H.rows() + ni.size() + fF.rows() + tF.rows()) {
                tF(row - H.rows() - ni.size() - fF.rows(), col) = value;
            } else if (row < H.rows() + ni.size() + fF.rows() + tF.rows() + tg.size()) {
                tg[row - H.rows() - ni.size() - fF.rows() - tF.rows()] = value;
            } else if (row < H.rows() + ni.size() + fF.rows() + tF.rows() + tg.size() + tH.rows()) {
                tH(row - H.rows() - ni.size() - fF.rows() - tF.rows() - tg.size(), col) = value;
            } else if (row < H.rows() + ni.size() + fF.rows() + tF.rows() + tg.size() + tH.rows() + fg.rows()) {
                fg(row - H.rows() - ni.size() - fF.rows() - tF.rows() - tg.size() - tH.rows(), col) = value;
            }
            col++;
        }
        row++;
    }

    std::vector<std::pair<int, double>> tb;

    for (int i = 0; i < 137; ++i) {
        if ((H.block(ni[i], 0, ni[i+1]-1 - ni[i], H.cols()) * xh).maxCoeff() <= 1e-8) {
            double tv = tF.row(i) * x + tg[i];
            tv += x.transpose() * tH.block((i-1) * nx, 0, nx, tH.cols()) * x;
            tb.push_back(std::make_pair(i, tv));
        }
    }

    Solution solution;
    solution.z = Eigen::VectorXd::Zero(nz);

    if (!tb.empty()) {
        auto min_pair = *std::min_element(tb.begin(), tb.end(),
            [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                return a.second < b.second;
            });

        solution.i = min_pair.first;
        solution.z = fF.block((solution.i-1) * nz, 0, nz, fF.cols()) * x + fg.block((solution.i-1) * nz, 0, nz, fg.cols());
    } else {
        solution.i = 0;
        solution.z = Eigen::VectorXd::Constant(nz, std::nan(""));
    }

    return solution;
}