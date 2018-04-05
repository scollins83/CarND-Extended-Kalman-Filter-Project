#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);

  //State parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Precompute some terms for reuse
  float term_1 = px * px + py * py;
  float term_2 = sqrt(term_1);
  float term_3 = (term_1 * term_2);

  // Avoid division by zero
  if(fabs(term_1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }
  // Compute the Jacobian Matrix
  Hj << (px / term_2), (py / term_2), 0, 0,
          -(py / term_1), (px / term_1), 0, 0,
          py * (vx * py - vy * px) / term_3, px * (px * vy - py * vx) / term_3, px / term_2, py / term_2;
  return Hj;
}
