#ifndef HELPER_H
#define HELPER_H

#include <iostream>
#include <chrono>
#include <queue> // std::queue
#include "Eigen/Core"
#include "Eigen/QR"

class Time_Difference
{
    std::chrono::steady_clock::time_point t_stamp;
    float t_total;
    size_t index;
    float decay;
    float weights;

  public:
    Time_Difference();
    void setDecay(float d) { decay = d; }
    size_t getStep() { return index; }

    float averageTimeDiff(std::chrono::steady_clock::time_point now);
};

struct PointsBuffer
{
    int num;
    std::queue<Eigen::VectorXd> x_buf;
    std::queue<Eigen::VectorXd> y_buf;

  public:
    PointsBuffer();
    void setBufferSize(int s) { num = s; }
    void updateBuffer();
    void getPoints();
};

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);

#endif