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

const double Lf = 2.67;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt) {
  // Create a new vector for the next state.
  Eigen::VectorXd next_state(state.size());

  auto x = state(0);
  auto y = state(1);
  auto psi = state(2);
  auto v = state(3);

  auto delta = actuators(0);
  auto a = actuators(1);

  // Recall the equations for the model:
  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
  // v_[t+1] = v[t] + a[t] * dt
  next_state(0) = x + v * cos(psi) * dt;
  next_state(1) = y + v * sin(psi) * dt;
  next_state(2) = psi + v / Lf * delta * dt;
  next_state(3) = v + a * dt;
  return next_state;
}

double steer_value=0;
double throttle_value=0;

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

double polyeval2(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (unsigned int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
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
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = (j[1]["psi"]);
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
		  
		  Eigen::VectorXd tmp_state(4);
		  Eigen::VectorXd actuator(2);
		  tmp_state << px,py,psi,v;
		  actuator << steer_value,throttle_value;
		  Eigen::VectorXd new_state(4);new_state = globalKinematic(tmp_state,actuator,0.1);
		  px = new_state[0];
		  py = new_state[1];
		  psi = new_state[2];
		  v = new_state[3];
		  
		  Eigen::VectorXd state(4);state << 0,0,0,v;
		  
		  
		  
		  Eigen::VectorXd x_ptx(ptsx.size());
		  Eigen::VectorXd y_ptx(ptsy.size());
		  
		  for(unsigned int idx=0;idx<ptsx.size();idx++)
		  {
			  double pt_d = sqrt(pow(ptsx[idx] - px,2)+pow(ptsy[idx] - py,2));
			  double ang = atan2(ptsy[idx] - py,ptsx[idx] - px);
			  ang -= psi;
			  double x_ = pt_d * cos(ang);
			  double y_ = pt_d*sin(ang);
			  x_ptx[idx] = x_;
			  y_ptx[idx] = y_;
		  }
		  
		  //Eigen::VectorXd poly_coeffs = polyfit(x_ptx,y_ptx,3);
		  Eigen::VectorXd poly_coeffs = polyfit(x_ptx,y_ptx,3);
		  
		  
		  vector<double> return_val = mpc.Solve(state,poly_coeffs);
		  steer_value = (return_val[0]);
		  throttle_value = return_val[1];
		  
		  std::cout << "steer_value" << (steer_value*-1)/deg2rad(25) << std::endl;
		  std::cout << "throttle_value" << throttle_value << std::endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          //msgJson["steering_angle"] = (steer_value*-1)/deg2rad(25);
		  msgJson["steering_angle"] = (steer_value*-1)/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
		  
		  for(unsigned int i=0;i<ptsx.size();i++)
		  {
			  double pt_d = sqrt(pow(ptsx[i] - px,2)+pow(ptsy[i] - py,2));
			  double ang = atan2((ptsy[i] - py),(ptsx[i] - px));
			  
			  ang -= psi;
			  double x_ = pt_d * cos(ang);
			  double y_ = pt_d*sin(ang);
			  next_x_vals.push_back(x_);
			  next_y_vals.push_back(y_);
		  }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


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
		  //TODO
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
