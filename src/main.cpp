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
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// for convenience
using json = nlohmann::json;


const double Lf = 2.67;
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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


std::vector<double> MapCarTransform(double mx,
                                    double my,
                                    const std::vector<double>& car_coords){
  double theta = -car_coords[2];

  double cx = car_coords[0];
  double cy = car_coords[1];

  std::vector<double> result{
      std::cos(theta) * (mx - cx) - std::sin(theta) * (my - cy),
      std::sin(theta) * (mx - cx) + std::cos(theta) * (my - cy)
  };
  return result;
}

std::vector<std::vector<double>> BatchMapCarTransform(const std::vector<double>& map_xcoords,
                                                      const std::vector<double>& map_ycoords,
                                                      const std::vector<double>& car_coords){
  std::vector<double> xresult;
  std::vector<double> yresult;
  for(int col = 0; col < map_xcoords.size(); col++){
    std::vector<double> single_result = MapCarTransform(map_xcoords[col], map_ycoords[col], car_coords);
    xresult.push_back(single_result[0]);
    yresult.push_back(single_result[1]);
  }
  return std::vector<std::vector<double>> {xresult, yresult};
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
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v *= 0.44704; // Meters per second correction
          double steer = j[1]["steering_angle"];
          steer *= -1.0;
          double throttle = j[1]["throttle"];

          int x_size = ptsx.size();
          int y_size = ptsy.size();

          assert(y_size == x_size);


          double pred_dt = 0.1;
          double delta_x = v * pred_dt * CppAD::cos(psi);

          double delta_x_car = v * pred_dt;
          double delta_y_car = v * pred_dt * CppAD::sin(v / 2 * Lf * steer * pred_dt);

          double delta_y = v * pred_dt * CppAD::sin(psi + v / 2 * Lf * steer * pred_dt);
          double delta_psi = v / Lf * steer * pred_dt;
          px += delta_x;
          py += delta_y;
          psi += delta_psi;
          v += throttle * pred_dt;

          std::vector<double> car_coords {px, py, psi};
          std::vector<std::vector<double>> transformed_waypoints = BatchMapCarTransform(ptsx, ptsy, car_coords);

          Eigen::VectorXd coeffs(3);
          Eigen::VectorXd xvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(transformed_waypoints[0].data(), transformed_waypoints[0].size());
          Eigen::VectorXd yvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(transformed_waypoints[1].data(), transformed_waypoints[1].size());
          coeffs = polyfit(xvals, yvals, 2);

          double epsi = -CppAD::Value(CppAD::atan(mpc.PolynomialValueOrDeriv(true, coeffs, 0.0)));
          double cte = -CppAD::Value(MPC::PolynomialValueOrDeriv(false, coeffs, 0.0));

          Eigen::VectorXd state(6);
          state << 0.0, 0.0, 0.0, v, cte, epsi;
          std::cout << "State: \n" << state <<  "\n";

          if(false) {
            double min_x = *std::min_element(std::begin(transformed_waypoints[0]), std::end(transformed_waypoints[0]));
            double diff = *std::max_element(std::begin(transformed_waypoints[0]), std::end(transformed_waypoints[0])) - min_x;

            std::vector<double> xgrid;
            std::vector<double> ygrid;
            std::cout << "begin grid print\n";
            for (int j = 0; j < 100; j++) {
              double x_tmp = min_x + j / 100.0 * diff;
              xgrid.push_back(x_tmp);
              double y_tmp = polyeval(coeffs, x_tmp);
              ygrid.push_back(y_tmp);
            }
            std::cout << "\nend grid print\n";


            plt::plot(xgrid, ygrid);
            plt::plot(transformed_waypoints[0], transformed_waypoints[1], "rx");
            plt::show();
          }

          std::vector<double> vars = mpc.Solve(state, coeffs);
          double steer_value = vars[6];
          double throttle_value = vars[7];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = - steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          mpc_x_vals.push_back(vars[0] + delta_x_car);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line


          mpc_x_vals.push_back(vars[8] + delta_x_car);
          mpc_x_vals.push_back(vars[9] + delta_x_car);
          // mpc_x_vals.push_back(vars[10] + delta_x_car);
          // mpc_x_vals.push_back(vars[11] + delta_x_car);
          // mpc_x_vals.push_back(vars[12]);

          mpc_y_vals.push_back(vars[1] + delta_y_car);
          mpc_y_vals.push_back(vars[15] + delta_y_car);
          mpc_y_vals.push_back(vars[16] + delta_y_car);
          // mpc_y_vals.push_back(vars[17] + delta_y_car);
          // mpc_y_vals.push_back(vars[18] + delta_y_car);
          // mpc_y_vals.push_back(vars[19]);

          // mpc_y_vals.push_back(vars[22]);
          // mpc_y_vals.push_back(vars[23]);
          // mpc_y_vals.push_back(vars[24]);
          // mpc_y_vals.push_back(vars[25]);
          // mpc_y_vals.push_back(vars[26]);
          // mpc_y_vals.push_back(vars[27]);

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals = transformed_waypoints[0];
          vector<double> next_y_vals = transformed_waypoints[1];

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


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
