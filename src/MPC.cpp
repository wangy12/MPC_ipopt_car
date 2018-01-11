#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

class FG_eval {
  public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {
    // implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    
    fg[0] = 0;


    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += W_cte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += W_epsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += W_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += W_delta * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += W_a * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += W_ddelta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += W_da * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //
    // Setup Constraints

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
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[3] * x0 * x0 * x0 + coeffs[2] * x0 * x0 + coeffs[1] * x0 + coeffs[0]; 
      AD<double> psides0 = CppAD::atan(3.0 * coeffs[3] * x0 * x0 + 2.0 * coeffs[2] * x0 + coeffs[1]);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * (-delta0) / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * (-delta0) / Lf * dt);
    }
  
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {

  const int n_vars = nb_states * N + nb_actuations * (N - 1);
  // Set the number of constraints
  const int n_constraints = nb_states * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  this->vars.resize(n_vars);
  
  for (int i = 0; i < n_vars; i++) {
    this->vars[i] = 0;
  }

  this->vars_lowerbound.resize(n_vars);
  this->vars_upperbound.resize(n_vars);
  
  // Set lower and upper limits for variables.
  
  for (int i = 0; i < delta_start; i++) {
    this->vars_lowerbound[i] = -1.0e19;
    this->vars_upperbound[i] = 1.0e19;
  }
  
  for (int i = delta_start; i < a_start; ++i) {
    this->vars_lowerbound[i] = -deg2rad(25);
    this->vars_upperbound[i] = deg2rad(25);
  }

  for (int i = a_start; i < n_vars; ++i) {
    this->vars_lowerbound[i] = -1.0;
    this->vars_upperbound[i] = 1.0;
  }
  
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  this->constraints_lowerbound.resize(n_constraints);
  this->constraints_upperbound.resize(n_constraints);
  
  for (int i = 0; i < n_constraints; i++) {
    this->constraints_lowerbound[i] = 0;
    this->constraints_upperbound[i] = 0;
  }
}

MPC::~MPC() {}

void MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  
  
  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];
  
  this->vars[x_start] = x;
  this->vars[y_start] = y;
  this->vars[psi_start] = psi;
  this->vars[v_start] = v;
  this->vars[cte_start] = cte;
  this->vars[epsi_start] = epsi;
  
  
  
  this->constraints_lowerbound[x_start] = x;
  this->constraints_lowerbound[y_start] = y;
  this->constraints_lowerbound[psi_start] = psi;
  this->constraints_lowerbound[v_start] = v;
  this->constraints_lowerbound[cte_start] = cte;
  this->constraints_lowerbound[epsi_start] = epsi;

  this->constraints_upperbound[x_start] = x;
  this->constraints_upperbound[y_start] = y;
  this->constraints_upperbound[psi_start] = psi;
  this->constraints_upperbound[v_start] = v;
  this->constraints_upperbound[cte_start] = cte;
  this->constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
  /*
  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start],   solution.x[a_start]};
          */
          
          
  this->steer = solution.x[delta_start];
  this->throttle = solution.x[a_start];

  this->x_vals = {};
  this->y_vals = {};

  for (int i = 0; i < N; ++i) {

    const double px = solution.x[x_start + i];
    const double py = solution.x[y_start + i];

    this->x_vals.emplace_back(px);
    this->y_vals.emplace_back(py);
  }

  
}
