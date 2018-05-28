#include "helper.h"

Time_Difference::Time_Difference()
{
    decay = 0.9;
    index = 0;
    t_stamp = std::chrono::steady_clock::now();
    t_total = 0;
    weights = 0.;
    default_td = 1 / 20.;

    printInfo = true;
    num = 5;
}

void Time_Difference::start()
{
    t_stamp = std::chrono::steady_clock::now();
}

float Time_Difference::averageTimeDiff(std::chrono::steady_clock::time_point current)
{
    if (index > 0)
    {
        float dif = std::chrono::duration_cast<std::chrono::microseconds>(current - t_stamp).count() / 1000. / 1000.;
        if (tbuf.size() < num)
        {
            tbuf.push_back(dif);
        }
        else
        {
            tbuf.pop_front();
            tbuf.push_back(dif);
        }

        t_total += dif;
        if (printInfo)
        {
            std::cout << "t_total = " << t_total  << std::endl;
        }

        float sum = 0.;
        for (size_t i = 0; i < tbuf.size(); i++)
        {
            sum += tbuf[i];
        }
        t_stamp = current;
        index++;
        return sum / tbuf.size();
    }
    else
    {
        t_stamp = current;
        index++;
        return default_td;
    }
}

float Time_Difference::weightedAverageTimeDiff(std::chrono::steady_clock::time_point current)
{
    if (index > 0)
    {
        t_total = t_total * decay + std::chrono::duration_cast<std::chrono::microseconds>(current - t_stamp).count() / 1000. / 1000.;
        t_stamp = current;
        weights = 1. + weights * decay;
        index++;

        if (printInfo)
        {
            std::cout << "t_total = " << t_total << " weights = " << weights << std::endl;
        }
        return t_total / weights;
    }
    else
    {
        t_stamp = current;
        index++;
        return default_td;
    }
}
/**
 * Update the buffer.
 * 
 * Push back the points or pop front before push back, depending on
 * the size of current buffer.
*/
void PointsBuffer::updateBuffer(const vector<double> &x, const vector<double> &y)
{
    if (x_buf.size() < num)
    {
        x_buf.push_back(x);
        y_buf.push_back(y);
    }
    else
    {
        x_buf.pop_front();
        y_buf.pop_front();

        x_buf.push_back(x);
        y_buf.push_back(y);
    }
}

/**
 * Get all points in the buffer
*/
void PointsBuffer::getPoints(vector<double> &x, vector<double> &y)
{
    size_t len = 0, total = 0;
    for (size_t i = 0; i < x_buf.size(); i++)
    {
        total += x_buf[i].size();
    }

    x.resize(total);
    y.resize(total);
    
    for (size_t i = 0; i < x_buf.size(); i++)
    {
        //x.resize(len + x_buf[i].size());
        //y.resize(len + y_buf[i].size());

        for (size_t j = 0; j < x_buf[i].size(); j++)
        {
            x[len + j] = x_buf[i][j];
            y[len + j] = y_buf[i][j];
        }
        len += x_buf[i].size();
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