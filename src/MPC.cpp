#include "MPC.h"
#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen/Core"
#include "Eigen/QR"

using CppAD::AD;

class FG_eval
{
public:
  Eigen::VectorXd coeffs;
  float Lf_;
  float ref_v_;
  size_t N_;
  float dt_;
  float weight_cte;
  // The solver takes all the state variables and actuator
  // variables in a singular vector. Thus, we should to establish
  // when one variable starts and another ends to make our lifes easier.
  size_t x_start;
  size_t y_start;
  size_t psi_start;
  size_t v_start;
  size_t cte_start;
  size_t epsi_start;
  size_t delta_start;
  size_t a_start;

  // Coefficients of the fitted polynomial.
  FG_eval(Eigen::VectorXd coeffs, float Lf, size_t N, float dt, float ref_v, float weight_cte_) : Lf_(Lf), N_(N), ref_v_(ref_v), dt_(dt), weight_cte(weight_cte_)
  {
    this->coeffs = coeffs;

    x_start = 0;
    y_start = x_start + N;
    psi_start = y_start + N;
    v_start = psi_start + N;
    cte_start = v_start + N;
    epsi_start = cte_start + N;
    delta_start = epsi_start + N;
    a_start = delta_start + N - 1;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector &fg, const ADvector &vars)
  {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int t = 0; t < N_; t++)
    {
      fg[0] += CppAD::pow(vars[cte_start + t], 2) * weight_cte;
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v_, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N_ - 1; t++)
    {
      fg[0] += 100. * CppAD::pow(vars[delta_start + t], 2);
      /** remove throtle! **/
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N_ - 2; t++)
    {
      fg[0] += 200. * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2) * 10;
      fg[0] += 50. * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
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
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N_; t++)
    {
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

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0);

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
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt_);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt_);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf_ * dt_);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt_);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt_));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf_ * dt_);
    }
  }
};

//
// MPC class definition
//

MPC::MPC() : N_(25), dt_(0.05), Lf_(2.67), ref_v_(50), time_shift(0), weight_cte(1.0)
{
  x_start = 0;
  y_start = x_start + N_;
  psi_start = y_start + N_;
  v_start = psi_start + N_;
  cte_start = v_start + N_;
  epsi_start = cte_start + N_;
  delta_start = epsi_start + N_;
  a_start = delta_start + N_ - 1;
}

MPC::~MPC() {}

vector<float> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs)
{
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = x0[0];
  double y = x0[1];
  double psi = x0[2];
  double v = x0[3];
  double cte = x0[4];
  double epsi = x0[5];

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N_ * 6 + (N_ - 1) * 2;
  // Number of constraints
  size_t n_constraints = N_ * 6;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
  {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++)
  {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++)
  {
    vars_lowerbound[i] = -1.;  //-1.0;
    vars_upperbound[i] = 1.75; //1.0;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs, Lf_, N_, dt_, ref_v_, weight_cte);

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
  //std::cout << "Cost " << cost << std::endl;
  /*
	for(int idx=0; idx<(2*N_ - 1); idx++)
	{
		std::cout<< solution.x[idx+1]<<",";
	}
	std::cout<< solution.x[2*N_]<<"\n";
*/
  std::vector<float> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  for (int i = 0; i < N_ * 2; i++)
  {
    result.push_back(solution.x[x_start + i]);
  }
  return result;
}
