#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <cmath>

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 5;
double dt = 0.3;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
// Miles per hour converted to m/s
const double ref_v = 0.44704 * 70;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t f_start = cte_start + N;
const size_t psides_start = f_start + N;
const size_t epsi_start = psides_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

// Evaluate a polynomial.
CppAD::AD<double> polyeval2(Eigen::VectorXd coeffs, CppAD::AD<double> x) {
  CppAD::AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result = result + coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

CppAD::AD<double> MPC::PolynomialValueOrDeriv(bool derivative, Eigen::VectorXd& coeffs, CppAD::AD<double> xval){
  size_t der_degree;
  if(derivative)
    der_degree = 1;
  else
    der_degree = 0;
  std::vector<double> my_coeffs;
  double xval_d = CppAD::Value(CppAD::Var2Par(xval));
  for(int i = 0; i < coeffs.rows(); i++)
    my_coeffs.push_back(coeffs(i, 0));
  CppAD::AD<double> result = CppAD::Poly(der_degree, my_coeffs, xval_d);
  return result;

}


class FG_eval {
public:
  Eigen::VectorXd coeffs;
  // Coefficients of the fitted polynomial.
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;


    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += 400.0 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 400.0 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 50.0 * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 200.0 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 40.0 * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 400.0 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 30.0 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + f_start] = vars[f_start];
    fg[1 + psides_start] = vars[psides_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> f1 = vars[f_start + t];
      AD<double> psides1 = vars[psides_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> f0 = vars[f_start + t - 1];
      AD<double> psides0 = vars[psides_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // AD<double> f0 = MPC::PolynomialValueOrDeriv(false, coeffs, x0);
      // AD<double> psides0 = CppAD::atan(MPC::PolynomialValueOrDeriv(true, coeffs, x0));

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = CppAD::Var2Par(psi1 - (psi0 + v0 * delta0 / Lf * dt));
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((y0 - f0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + f_start + t] =
          f1 - polyeval2(coeffs, x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + psides_start + t] =
          psides1 - CppAD::atan(coeffs[1] * 2 + coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};


//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = x0[0];
  double y = x0[1];
  double psi = x0[2];
  double v = x0[3];
  double cte = x0[4];
  double f = -cte;
  double epsi = x0[5];
  double psides = psi - epsi;

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 8 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 8;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  // double pred_dt = 0.1;

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[f_start] = f;
  vars[psides_start] = psides;
  vars[epsi_start] = epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[f_start] = f;
  constraints_lowerbound[psides_start] = psides;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[f_start] = f;
  constraints_upperbound[psides_start] = psides;
  constraints_upperbound[epsi_start] = epsi;


  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  //
  // Check some of the solution values
  //
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  int offset = 1;

  std::cout << "Let us inspect the solution: \n";
  for(int i = 0; i < N - 1; i++){
    std::cout << solution.x[x_start + i] << " ";
    std::cout << solution.x[y_start + i] << " ";
    std::cout << solution.x[psi_start + i] << " ";
    std::cout << solution.x[v_start + i] << " ";
    std::cout << solution.x[cte_start + i] << " ";
    std::cout << solution.x[f_start + i] << " ";
    std::cout << solution.x[psides_start + i] << " ";
    std::cout << solution.x[epsi_start+ i] << " ";
    std::cout << solution.x[delta_start+ i] << " ";
    std::cout << solution.x[a_start+ i] << " ";
    std::cout << "\n";
  }


  return {solution.x[x_start + offset], solution.x[y_start + offset],
          solution.x[psi_start + offset], solution.x[v_start + offset],
          solution.x[cte_start + offset], solution.x[epsi_start + offset],
          solution.x[delta_start],   solution.x[a_start], solution.x[x_start + offset + 1],
          solution.x[x_start + offset + 2], solution.x[x_start + offset + 3],
          solution.x[x_start + offset + 4], solution.x[x_start + offset + 5],
          solution.x[x_start + offset + 6], solution.x[x_start + offset + 7], solution.x[y_start + offset + 1],
          solution.x[y_start + offset + 2], solution.x[y_start + offset + 3],
          solution.x[y_start + offset + 4], solution.x[y_start + offset + 5],
          solution.x[y_start + offset + 6], solution.x[y_start + offset + 7],
          CppAD::Value(MPC::PolynomialValueOrDeriv(false, coeffs, solution.x[x_start + offset])),
          CppAD::Value(MPC::PolynomialValueOrDeriv(false, coeffs, solution.x[x_start + offset + 1])),
          CppAD::Value(MPC::PolynomialValueOrDeriv(false, coeffs, solution.x[x_start + offset + 2])),
          CppAD::Value(MPC::PolynomialValueOrDeriv(false, coeffs, solution.x[x_start + offset + 3])),
          CppAD::Value(MPC::PolynomialValueOrDeriv(false, coeffs, solution.x[x_start + offset + 4])),
          CppAD::Value(MPC::PolynomialValueOrDeriv(false, coeffs, solution.x[x_start + offset + 5])),
          CppAD::Value(MPC::PolynomialValueOrDeriv(false, coeffs, solution.x[x_start + offset + 6]))

  };
}


