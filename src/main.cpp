#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include "behavior.h"
#include "json.hpp"
#include "localization.h"
#include "path.h"
#include "trajectory_generator.h"

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
          // std::cout << "sensor_fusion: " << sensor_fusion.dump() << "\n";
          std::cout << "{";
          std::cout << "\"timestamp\": "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(
                      start.time_since_epoch()
                    ).count();

          json j_prev;
          j_prev["prev_x"] = previous_path_x;
          j_prev["prev_y"] = previous_path_y;
          std::cout << ", \"prev_x\": " << j_prev["prev_x"].dump();
          std::cout << ", \"prev_y\": " << j_prev["prev_y"].dump();

          Localization localization = {
              .x = j[1]["x"],
              .y = j[1]["y"],
              .s = j[1]["s"],
              .d = j[1]["d"],
              .yaw = j[1]["yaw"],
              .speed = j[1]["speed"],
          };
          planner.set_localization(localization);

          std::vector<Obstacle> obstacles;
          for (int i = 0; i < sensor_fusion.size(); i++) {
            obstacles.push_back(Obstacle{
                .id = sensor_fusion[i][0],
                .x = sensor_fusion[i][1],
                .y = sensor_fusion[i][2],
                .v_x = sensor_fusion[i][3],
                .v_y = sensor_fusion[i][4],
                .s = sensor_fusion[i][5],
                .d = sensor_fusion[i][6],
            });
          }
          planner.set_obstacles(obstacles);
          Trajectory trajectory_prev = {
            .x = previous_path_x,
            .y = previous_path_y,
          };

          Path path = planner.next(generator, trajectory_prev, end_path_s);

          json msgJson;
          msgJson["next_x"] = path.trajectory.x;
          msgJson["next_y"] = path.trajectory.y;

          json j;
          j["next_s"] = path.s;
          j["next_d"] = path.d;
          std::cout << ", \"next_s\": " << j["next_s"].dump();
          std::cout << ", \"next_d\": " << j["next_d"].dump();

          std::cout<< ", \"next_x\": " << msgJson["next_x"].dump();
          std::cout<< ", \"next_y\": " << msgJson["next_y"].dump();

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          auto end = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsed = end - start;
          std::cout << ", \"took\": " << elapsed.count() * 1000;
          int sleep_time = 100 - (elapsed.count() * 1000);
          std::cout << ", \"sleep\": " << sleep_time;

          std::cout << "}" << "\n";

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
    //std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    //std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
