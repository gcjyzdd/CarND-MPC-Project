#include "helper.h"

Time_Difference::Time_Difference()
{
    decay = 0.9;
    index = 0;
    t_stamp = std::chrono::steady_clock::now();
    t_total = 0;
    weights = 0.;
    default_td = 1 / 20.;
}

void Time_Difference::start()
{
    t_stamp = std::chrono::steady_clock::now();
}

float Time_Difference::averageTimeDiff(std::chrono::steady_clock::time_point current)
{
    if (index > 0)
    {
        t_total = t_total * decay + std::chrono::duration_cast<std::chrono::seconds>(current - t_stamp).count();
        t_stamp = current;
        weights = 1 + weights * decay;
        index++;

        return t_total / weights;
    }
    else
    {
        t_stamp = current;
        index++;
        return default_td;
    }
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}