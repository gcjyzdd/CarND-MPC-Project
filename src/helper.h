#ifndef HELPER_H
#define HELPER_H

#include <iostream>
#include <chrono>
#include <deque> // std::deque
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"

using std::vector;

class Time_Difference
{
  std::chrono::steady_clock::time_point t_stamp;
  float t_total;
  size_t index;
  float decay;
  float weights;
  float default_td;
  bool printInfo;
  int num;
  std::deque<float> tbuf;

public:
  Time_Difference();
  void start();
  void setNum(int n){num = n;}
  void setDefaultTimeDifference(const float td) { default_td = td; }
  void setDecay(const float d) { decay = d; }
  size_t getStep() { return index; }

  float averageTimeDiff(std::chrono::steady_clock::time_point now);
  float weightedAverageTimeDiff(std::chrono::steady_clock::time_point now);
};

struct PointsBuffer
{
  int num;
  std::deque<vector<double>> x_buf;
  std::deque<vector<double>> y_buf;

public:
  PointsBuffer() { num = 4; };
  void setBufferSize(const int s) { num = s; }
  void updateBuffer(const vector<double> &x, const vector<double> &y);
  void getPoints(vector<double> &x, vector<double> &y);
};

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);

struct WayPoint
{
  double x,y;
};

struct by_x
{
  bool operator()(WayPoint const &a, WayPoint const &b)
  {
    return a.x < b.x;
  }
};

#endif