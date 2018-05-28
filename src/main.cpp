#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include "Eigen-3.3/Eigen/Core"

#include "MPC.h"
#include "helper.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  // set mpc configuration
  mpc.Lf_ = 1.67;
  mpc.setHorizon(8);
  mpc.ref_v_ = 130 * 0.44704; // m/s
  mpc.dt_ = 1 / 3.;
  mpc.a_max = 2.0;
  mpc.a_min = -1.0;

  MPCWeights mw;
  mw.w_cte = 180.;
  mw.w_epsi = 200.;
  mw.w_steer_dif = 80;
  mw.w_steer = 5;
  mw.w_a = 0;
  mw.w_a_dif = 0.;
  mw.w_v = 1.2;

  mpc.w = mw;

  Time_Difference td;
  td.setNum(5);
  PointsBuffer ptsBuffer;
  ptsBuffer.setBufferSize(5);

  h.onMessage([&mpc, &ptsBuffer, &td](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);

    //begin = end;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
    {
      string s = hasData(sdata);
      if (s != "")
      {
        //cout << sdata << endl;

        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          v = v * 0.44704; // velocity in m/s

          double delta = j[1]["steering_angle"];
          double acc = j[1]["throttle"];
          delta = -delta;
          acc *= mpc.a_max;
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          ptsBuffer.updateBuffer(ptsx, ptsy);
          ptsBuffer.getPoints(ptsx, ptsy);
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
          // sort points by x
          std::sort(wpts.begin(), wpts.end(), by_x());
          for (size_t i = 0; i < ptsx.size(); i++)
          {
            vx[i] = wpts[i].x;
            vy[i] = wpts[i].y;
          }
          Eigen::VectorXd state(6);
          Eigen::VectorXd coeffs = polyfit(vc_x, vc_y, 3);
          //std::cout << "coeffs" << coeffs << std::endl;
          double Latency = 0.1; //0.1;//0.1;     // 100 ms
          double Lf = mpc.Lf_;

          double x0 = 0, y0 = 0, psi0 = 0, v0 = v, cte0 = polyeval(coeffs, x0) - y0;
          double epsi0 = psi0 - atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);

          double x1 = x0 + Latency * v0; // + 0.5 * Latency * Latency * acc;
          double psi1 = psi0 + v0 * delta / Lf * Latency;
          double y1 = y0; // + v * Latency * psi1 / 2.;
          double v1 = v0 + acc * Latency;
          double cte1 = cte0 + v0 * sin(epsi0) * Latency;
          double epsi1 = epsi0 + v0 * delta * Latency / Lf;

          state << x1, y1, psi1, v1, cte1, epsi1;

          float dt = td.averageTimeDiff(std::chrono::steady_clock::now()) - Latency;
          std::cout << "dt = " << dt << " fr = " << 1 / dt << " v = " << v << std::endl;
          //mpc.dt_ = dt * 7; //(time_dif_total / index) / 1000. - Latency;
          auto result = mpc.Solve(state, coeffs);
          double steer_value = result[0];
          double throttle_value = result[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value / deg2rad(25.);
          msgJson["throttle"] = throttle_value > 0 ? throttle_value / mpc.a_max : -throttle_value / mpc.a_min;

          int N = mpc.N_;
          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals(result.begin() + 2, result.begin() + 2 + N - 1);
          vector<double> mpc_y_vals(result.begin() + 2 + N, result.begin() + 2 + N * 2 - 1);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          //vector<double> next_x_vals;
          //vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = vx;
          msgJson["next_y"] = vy;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds((int)(1000 * Latency)));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
