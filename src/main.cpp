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
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

bool is_car_in_front(vector<vector<double>> sensor_fusion, double car_s, int lane, int prev_size) {
  int lane_center = lane * 4 + 2;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    float d = sensor_fusion[i][6];
    if ((lane_center - 2) < d && d < (lane_center + 2)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double)prev_size * 0.02 * check_speed);
      if ((check_car_s >= car_s) && (check_car_s - car_s <= 30)) {
        return true;
      }
    }
  }

  return false;
}

bool is_lane_empty(vector<int> edges, vector<vector<double>> sensor_fusion, double car_s, int prev_size) {
  for (int i = 0; i < sensor_fusion.size(); i++) {
    float d = sensor_fusion[i][6];
    if (edges[0] < d && d < edges[1]) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s_from = sensor_fusion[i][5];

      double check_car_s_to = check_car_s_from + ((double)prev_size * 0.02 * check_speed);
			// Check if a car or its predicted position are out of buffer zone
      if ((car_s - 10 <= check_car_s_from && check_car_s_from <= car_s + 30) ||
          (car_s - 10 <= check_car_s_to && check_car_s_to <= car_s + 30)) {
        return false;
      }
    }
  }

  return true;
}

bool is_left_empty(vector<vector<double>> sensor_fusion, double car_s, int lane, int prev_size) {
  if (lane == 0) {
    return false;
  }

  int lane_center = lane * 4 + 2;
  vector<int> edges = {lane_center - 6, lane_center - 2};
  return is_lane_empty(edges, sensor_fusion, car_s, prev_size);
}

bool is_right_empty(vector<vector<double>> sensor_fusion, double car_s, int lane, int prev_size) {
  if (lane == 2) {
    return false;
  }

  int lane_center = lane * 4 + 2;
  vector<int> edges = {lane_center + 2, lane_center + 6};
  return is_lane_empty(edges, sensor_fusion, car_s, prev_size);
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

  int lane = 1;
  double ref_vel = 0.0;

  h.onMessage([&ref_vel, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          if (is_car_in_front(sensor_fusion, car_s, lane, prev_size)) {
            if (is_left_empty(sensor_fusion, car_s, lane, prev_size)) {
              lane -= 1;
            } else if (is_right_empty(sensor_fusion, car_s, lane, prev_size)) {
              lane += 1;
            } else {
							// Deccelerate
              ref_vel -= 0.224;
            }
          } else if (ref_vel < 49.0) {
						// Accelerate up to the speed limit
            ref_vel += 0.224;
          }

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          std::vector<double> ptsx;
          std::vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw * M_PI / 180.0;

          if (prev_size < 2) {
						// It's the initial state, need to figure out the car angle
            double x_prev = ref_x - cos(ref_yaw);
            double y_prev = ref_y - sin(ref_yaw);
            ptsx.push_back(x_prev);
            ptsy.push_back(y_prev);

            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
          } else {
						// Take the first two points of the previous path to keep the spline smooth
            double x_prev = previous_path_x[prev_size - 2];
            double y_prev = previous_path_y[prev_size - 2];
            ptsx.push_back(x_prev);
            ptsy.push_back(y_prev);

            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);

            ref_yaw = atan2(ref_y - y_prev, ref_x - x_prev);
          }

					// Using 3 waypoints, 30m spaced to keep the spline smooth
          std::vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          std::vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

          double x_add_on = 0;

          for (int i = 1; i <= 50 - prev_size; i++) {
            double N = (target_dist / (0.02 * ref_vel / 2.24));
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
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
