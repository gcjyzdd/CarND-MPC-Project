# Report of Self-Driving-Car Project: Model Predictive Control

<script src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.0/MathJax.js?config=TeX-AMS-MML_HTMLorMML" type="text/javascript"></script>

## Introduction

In this project, a MPC was implemented to drive a car around the track in the car simulator. Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.


## Model

In this project, a motion bicycle model was adoptted. The state space representation of the system is:

<div style="text-align:center"><img src ='./images/Screenshot from 2018-05-28 20-04-39.png' /></div>

where 

* $$x$$ is the longitudinal coordinate
* $$y$$ is the lateral coordinate
* $$\psi$$ is the yaw angle of the car body
* $$v$$ is the velocity of the car
* $$cte$$ is the cross track error
* and $$e_{\psi}$$ is the error of the yaw angle according to the desired trajectory.

The control inputs of the car is $$\delta$$ and $$a$$. $$\delta$$ is the yaw angle (steer angle) of the front wheels and $$a$$ is the accleration.

According to the simulator, the steer angle is within [-25, 25] degree. To get the maximum accleration, set the throtle to 1 for a while and get the difference of velocity. Since $$a = \frac{dv}{dt}$$, the maximum accleration is around $$2m/s^2$$.


## Sample Time and Prediction Horizon

Ideally, the vehicle model should match real world physics as much as possible. That said, the discrtized time step (sample time) should be as small as possible so that the discretization error is negligible. 

The predicted distance is roughly $$v\times N\times dt$$; in order to go through the bend road smoothly, the predicted distance should be around 20 metres. Assuming the car is driving at 50km/h, the total predicted time should be around $$20\times3.6/50=1.44s$$.

However, we are driving the simulated car instead of a real car. The car simulator updates states of the car with variable time steps. Open the file `ProjectSettings/TimeManager.asset` of the simulator repo, we find

```
%YAML 1.1
%TAG !u! tag:unity3d.com,2011:
--- !u!5 &1
TimeManager:
  m_ObjectHideFlags: 0
  Fixed Timestep: .0199999996
  Maximum Allowed Timestep: .333333343
  m_TimeScale: 1
```

where the maximum `fps` is 1/0.02 = 50 and the minimum `frs` is 1/0.333=3.

The sample time of MPC should match the time step that the simulator updates the control inputs of the car. I tried the sample time with `1/50, 1/20, 1/10, 1/8, 1/5, 1/3` second and it turned out that `1/3` performed the best. And the prediction horizon is 8; as a result, the total prediction time is `8/3=2.7` second.

To make the mpc solver more stable and faster, a buffer was implemented to store previous solution. See `line 58` of [MPC.h](./src/MPC.h) and `line 213 and 301-304` of [MPC.cpp](./src/MPC.cpp):

```cpp
// MPC.h
  std::vector<float> var_init;

// MPC.cpp
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
  {
    vars[i] = var_init[i];//0.0;
  }

  for (size_t i = 0; i < var_init.size(); i++)
  {
    var_init[i] = solution.x[i];
  }
```

The configuration of the MPC controller was set at `line 48-54` of [main.cpp](./src/main.cpp):

``` cpp
  // set mpc configuration
  mpc.Lf_ = 1.67;
  mpc.setHorizon(8);
  mpc.ref_v_ = 100 * 0.44704; // m/s
  mpc.dt_ = 1 / 3.;
  mpc.a_max = 2.0;
  mpc.a_min = -1.0;
```

## Polynomial Fitting and MPC Preprocessing

To get the polynomial coefficients of the desired trajectory, first tansfer world coordinate way point positions to vehicle coordinate. Because

$$p^W = R^vp^v+T^v$$,

we get

$$p^v = (R^v)^T (p^W-T^v)$$. $$p^W$$ is world coordnate position and $$p^v$$ is vehicle coordinate position, and $$R^v, T^v$$ are the rotation and translation component of the vehicle.

This part was implemented in `line 113-124` of [main.cpp](./src/main.cpp):

```cpp
          vector<WayPoint> wpts(ptsx.size());
          vector<double> vx(ptsx.size()), vy(ptsx.size()); // way points in vehicle coordinate system
          Eigen::VectorXd vc_x(ptsx.size());
          Eigen::VectorXd vc_y(ptsx.size());
          // P_v = R_c' * (P_w - P_c)
          for (size_t i = 0; i < ptsx.size(); i++)
          {
            wpts[i].x = cos(psi) * (ptsx[i] - px) + sin(psi) * (ptsy[i] - py);
            wpts[i].y = -sin(psi) * (ptsx[i] - px) + cos(psi) * (ptsy[i] - py);
            vc_x[i] = wpts[i].x;
            vc_y[i] = wpts[i].y;
          }
```

## Model Predictive with Latency

## Demo Simulation

