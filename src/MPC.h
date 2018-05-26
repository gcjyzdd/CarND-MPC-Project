#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen/Core"

using namespace std;

struct MPCWeights
{
  float w_cte;
  float w_steer;
  float w_a;
  float w_v;  
  float w_epsi;
  float w_steer_dif;
  float w_a_dif;
  MPCWeights()
  {
    w_a = 1;
    w_a_dif = 1;
    w_cte = 1;
    w_epsi = 1;    
    w_steer = 1;
    w_steer_dif = 1;
    w_v = 1;
  }
};

class MPC
{
public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state.
  // Return the next state and actuations as a
  // vector.
  void setHorizon(int h);
  vector<float> Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs);

public:
  // We set the number of timesteps to 25
  // and the timestep evaluation frequency or evaluation
  // period to 0.05.
  size_t N_;
  float dt_;

  size_t time_shift;
  MPCWeights w;

  float a_max;
  float a_min;
  float steer_max;
  float steer_min;
  
  std::vector<float> var_init;
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