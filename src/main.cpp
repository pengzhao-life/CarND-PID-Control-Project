#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */

  // twiddle pid
  // xx, yy, zz
  pid.Init(0.2, 0.001, 3.0);
  pid.InitDP({0.01, 0.001, 0.1});
  // set twiddle = false to turn off twiddle for parameters optimization
  bool twiddle = true;
  double tolerence = 0.02;
  long counter = 0;
  long tmp_counter = -1;
  int n = 100;
  double error = 0.0;
  double best_error  = std::numeric_limits<double>::max();
  // index 0 for p, 
  // index 1 for i.
  // index 2 for d.
  int index =0;
  // step 0 for p[i] += dp[i], i.e. +
  // step 1 for p[i] -= 2 * dp[i], i.e. -
  // step 2 for p[i] += dp[i] (), i.e. no change
  int step = -1;

  h.onMessage([&pid, &twiddle, &counter, &tmp_counter, &n, 
  &best_error, &error, &tolerence, &index, &step](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          //double speed = std::stod(j[1]["speed"].get<string>());
          //double angle = std::stod(j[1]["steering_angle"].get<string>());
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          counter++;
          std::cout << std::to_string(counter) << std::endl;
          double steer_value = 0.0;

          if(twiddle){
            error += cte * cte;
           
            if(step == -1 && counter == n){
              // get the error for the first n data
              best_error = error;
              // prepare for 'p' controller
              index = 0;
              // prepare for p[i] += dp[i]
              step = 0;
              pid.UpdateParameter(index, 1.0);

            }else if(pid.DPSum() > tolerence){
              if(step == 0 && counter == n){
                if(error < best_error){
                  best_error = error;
                  pid.UpdateDP(index, 1.1);
                  // prepare for next controller
                  step = 0;
                  index = (index+1)%3;
                }else{
                  pid.UpdateParameter(index, -2.0);
                  // prepare for p[i] -= 2 * dp[i]
                  step = 1;
                }
              }else if(step == 1 && counter == n){
                if(error < best_error){
                  best_error = error;
                  pid.UpdateDP(index, 1.1);
                  // prepare for next controller
                  step = 0;
                  index = (index+1)%3;
                }else{
                  pid.UpdateParameter(index, 1.0);
                  pid.UpdateDP(index, 0.9);
                }          
              }
            }
            if(counter == n){
              error = 0.0;
              counter = 0;
              pid.PrintPID();
            }
          }

          pid.UpdateError(cte);
          steer_value = 0.0 - pid.TotalError();

          // Clip steer_value to [-1,1]
          // We can use std::clamp if c++ 17 is supported
          if (steer_value > 1) steer_value = 1;
          if(steer_value < -1) steer_value = -1;
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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