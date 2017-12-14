#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
using CppAD::AD;

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  static CppAD::AD<double> PolynomialValueOrDeriv(bool derivative, Eigen::VectorXd& coeffs, CppAD::AD<double> xval);
  static double NormalizedAngle(double angle);
  static double polyeval(Eigen::VectorXd coeffs, double x);
};

#endif /* MPC_H */
