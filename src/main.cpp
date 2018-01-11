#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"]; // The global x positions of the waypoints.
          vector<double> ptsy = j[1]["ptsy"]; // The global y positions of the waypoints. This corresponds to the z coordinate in Unity since y is the up-down direction.
          
          double px = j[1]["x"]; // The global x position of the vehicle.
          double py = j[1]["y"]; // The global y position of the vehicle.
          
          double psi = j[1]["psi"]; // The orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
          
          double v = j[1]["speed"]; // The current velocity in mph.
          
          double delta = j[1]["steering_angle"]; // The current steering angle in radians.
          double a = j[1]["throttle"]; // The current throttle value [-1, 1].

		//			psi

		//            90
		//
		//  180                   0/360
		//
		//            270

		  
		  const int nb_waypoints = ptsx.size();
		  
		  // Convert waypoints to vehicle space from global space
		  Eigen::VectorXd waypoints_xs(nb_waypoints);
          Eigen::VectorXd waypoints_ys(nb_waypoints);

          for(int i = 0; i < nb_waypoints; ++i) {

            const double dx = ptsx[i] - px;
            const double dy = ptsy[i] - py;

            waypoints_xs[i] = dx * cos(-psi) - dy * sin(-psi);
            waypoints_ys[i] = dy * cos(-psi) + dx * sin(-psi);
          }
          
          // Fit polynomial
          const int order = 3;
          auto coeffs = polyfit(waypoints_xs, waypoints_ys, order);
          
          // Points to display
          std::vector<double> next_xs(N);
          std::vector<double> next_ys(N);
          const double D = 5.0;

          for (int i = 0; i < N; ++i) {

            const double dx = D * i;
            const double dy = coeffs[3] * dx * dx * dx + coeffs[2] * dx * dx + coeffs[1] * dx + coeffs[0];

            next_xs[i] = dx;
            next_ys[i] = dy;
          }
          
          // error estimation
          // current CTE is fitted polynomial (road curve) evaluated at px = 0.0
          const double cte = coeffs[0];

          // current heading error epsi is the tangent to the road curve at px = 0.0
          // epsi = arctan(f') where f' is the derivative of the fitted polynomial
          // f' = 3.0 * K[3] * px0 * px0 + 2.0 * K[2] * px0 + K[1]
          const double epsi = -atan(coeffs[1]);


          // current state must be in vehicle coordinates with the delay factored in
          // kinematic model is at play here
          // note that at current state at vehicle coordinates:
          // px, py, psi = 0.0, 0.0, 0.0
          // note that in vehicle coordinates it is going straight ahead the x-axis
          // which means position in vehicle's y-axis does not change
          // the steering angle is negative the given value as we have
          // as recall that during transformation we rotated all waypoints by -psi
          const double current_px = 0.0 + v * dt;
          const double current_py = 0.0;
          const double current_psi = 0.0 + v * (-delta) / Lf * dt;
          const double current_v = v + a * dt;
          const double current_cte = cte + v * sin(epsi) * dt;
          const double current_epsi = epsi + v * (-delta) / Lf * dt;

          
          Eigen::VectorXd state(nb_states);
          state << current_px, current_py, current_psi, current_v, current_cte, current_epsi;

          
          
          // MODEL PREDICTIVE CONTROL
          
          mpc.Solve(state, coeffs);
          
          

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = mpc.steer;
          msgJson["throttle"] = mpc.throttle;

          //Display the MPC predicted trajectory 
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc.x_vals;
          msgJson["mpc_y"] = mpc.y_vals;

          

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_xs;
          msgJson["next_y"] = next_ys;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
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
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
