#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Initialize local variables
  VectorXd rmse(4);
  rmse.fill(0);
  int num_points = estimations.size();

  // Determine if we can proceed with the current vectors
  if (num_points == 0 || num_points != ground_truth.size()) {
    std::cout << "ERROR: Invalid vector size encountered during RMSE calculation. Either there is nothing to calculate or the vectors are not the same size." << std::endl;
    return rmse;
  }

  // Get sum of squared errors
  for (int i = 0; i < num_points; ++i) {
    VectorXd diff = estimations[i] - ground_truth[i];
    diff = diff.array().square();
    rmse += diff;
  }

  // Calculate mean
  rmse /= (double) num_points;

  // Calculate sqrt of MSE
  rmse = rmse.array().sqrt();

  return rmse;
}
