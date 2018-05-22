#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen/Core"

using namespace std;

class MPC
{
public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state.
  // Return the next state and actuations as a
  // vector.
  vector<float> Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs);

public:
  // We set the number of timesteps to 25
  // and the timestep evaluation frequency or evaluation
  // period to 0.05.
  size_t N_;
  float dt_;

  size_t time_shift;
  float weight_cte;

  // It was obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on a
  // flat terrain.
  //
  // Lf was tuned until the the radius formed by the simulating the model
  // presented in the classroom matched the previous radius.
  //
  // This is the length from front to CoG that has a similar radius.
  float Lf_;

  // The reference velocity  mph.
  float ref_v_;

  size_t x_start;
  size_t y_start;
  size_t psi_start;
  size_t v_start;
  size_t cte_start;
  size_t epsi_start;
  size_t delta_start;
  size_t a_start;
};

double polyeval(Eigen::VectorXd coeffs, double x);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);
#endif /* MPC_H */