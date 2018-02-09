#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

typedef CPPAD_TESTVECTOR(double) Dvector;

using namespace std;

//  Set the timestep length and duration
const int N = 20;
const double dt = 0.1;

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

// The reference velocity 
const double ref_v = 76.0;

const int nb_states =  6; // number of state 
const int nb_actuations = 2; // actuation variables

const int x_start = 0;
const int y_start = x_start + N;
const int psi_start = y_start + N;
const int v_start = psi_start + N;
const int cte_start = v_start + N;
const int epsi_start = cte_start + N;
const int delta_start = epsi_start + N;
const int a_start = delta_start + N - 1;

// weights for cost function
const double W_cte = 20.0;
const double W_epsi = 20.0;
const double W_v = 0.01;
const double W_delta = 0.05;
const double W_a = 0.05;
const double W_ddelta = 2.0; // weight for high difference between consecutive steering actuations
const double W_da = 0.1; // weight for high difference between consecutive acceleration actuations

class MPC {
 public:
   double steer;
   double throttle;

   Dvector vars; // where all the state and actuation variables will be stored
   Dvector vars_lowerbound; //lower limit for each corresponding variable in x
   Dvector vars_upperbound; //upper limit for each corresponding variable in x
   Dvector constraints_lowerbound; // value constraint for each corresponding constraint expression
   Dvector constraints_upperbound; // value constraint for each corresponding constraint expression

   std::vector<double> x_vals;
   std::vector<double> y_vals;
 
 
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
