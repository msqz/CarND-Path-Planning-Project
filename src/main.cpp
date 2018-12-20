#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "localization.h"
#include "behavior.h"
#include "trajectory_generator.h"
#include "path.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  Map map = {
    .s = map_waypoints_s,
    .x = map_waypoints_x,
    .y = map_waypoints_y,
    .dx = map_waypoints_dx,
    .dy = map_waypoints_dy,
  };
  TrajectoryGenerator generator(map);
  BehaviorPlanner planner;

  h.onMessage([&planner, &generator](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                                 uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          auto start = std::chrono::system_clock::now();
          // j[1] is the data JSON object

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          json msgJson;

          Localization localization = {
              .x = j[1]["x"],
              .y = j[1]["y"],
              .s = j[1]["s"],
              .d = j[1]["d"],
              .yaw = j[1]["yaw"],
              .speed = j[1]["speed"],
              .prev_path_s = end_path_s,
              .prev_path_d = end_path_d,
          };
          if (previous_path_x.size() == 0){
            localization.prev_path_s = localization.s;
            localization.prev_path_d = localization.d;
          }

          planner.set_localization(localization);          
          Path path = planner.next();
          Trajectory trajectory = generator.generate(path);
                            
          vector<double> next_x_vals;
          vector<double> next_y_vals;
    
          if (previous_path_x.size() > 0) {
            next_x_vals.push_back(previous_path_x[0]);
            next_y_vals.push_back(previous_path_y[0]);
          } else {
            next_x_vals.push_back(trajectory.x[0]);
            next_y_vals.push_back(trajectory.y[0]);
          }

          for (int i = 1; i < trajectory.size(); i++) {
            double delta_x = trajectory.x[i] - trajectory.x[i-1];
            double delta_y = trajectory.y[i] - trajectory.y[i-1];

            next_x_vals.push_back(next_x_vals[i-1] + delta_x);
            next_y_vals.push_back(next_y_vals[i-1] + delta_y);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          json j;
          // j["next_s"] = path.s;
          // j["next_d"] = path.d;
          // std::cout << "next_s: " << j["next_s"].dump() << "\n";
          // std::cout << "next_d: " << j["next_d"].dump() << "\n";
          // std::cout << "next_x: " << msgJson["next_x"].dump() << "\n";
          // std::cout << "next_y: " << msgJson["next_y"].dump() << "\n";

          auto msg = "42[\"control\"," + msgJson.dump() + "]";
          auto end = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsed = end-start;
          std::cout << "took: " << elapsed.count() * 1000 << "ms\n";

          int sleep_time = 1000 - (elapsed.count()*1000);
          std::cout<<"sleep for: " << sleep_time << "\n";
          this_thread::sleep_for(chrono::milliseconds(sleep_time));
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
