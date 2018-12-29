#ifndef STATE_H
#define STATE_H

#include <math.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include "constraints.h"
#include "json.hpp"
#include "localization.h"
#include "path.h"

using json = nlohmann::json;

std::map<std::string, std::vector<std::string>> STATES_S = {
    {"STOP", {"ACC", "STOP"}},
    {"ACC", {"CRUISE", "ACC", "DECC"}},
    {"DECC", {"CRUISE", "DECC", "ACC", "STOP"}},
    {"CRUISE", {"ACC", "DECC", "CRUISE"}},
};

std::map<std::string, std::vector<std::string>> STATES_D = {
    {"STRAIGHT", {"STRAIGHT", "RIGHT"}},
    {"RIGHT", {"STRAIGHT", "RIGHT"}},
};

std::vector<double> JMT(std::vector<double> start, std::vector<double> end, double t) {
  double a_0 = start[0];
  double a_1 = start[1];
  double a_2 = start[2] / 2.0;
  double c_0 = a_0 + (a_1 * t) + (a_2 * pow(t, 2));
  double c_1 = a_1 + (2 * a_2 * t);
  double c_2 = 2 * a_2;

  Eigen::Matrix3d A;
  A << pow(t, 3), pow(t, 4), pow(t, 5),
      3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
      6 * t, 12 * pow(t, 2), 20 * pow(t, 3);

  Eigen::Vector3d B{end[0] - c_0, end[1] - c_1, end[2] - c_2};

  Eigen::MatrixXd a_345 = A.inverse() * B;
  std::vector<double> alphas{a_0, a_1, a_2, a_345(0), a_345(1), a_345(2)};

  return alphas;
}

class State {
 public:
  virtual Path build_path(const Localization localization){};
};

class AccState : public State {
 public:
  Path build_path(const Localization localization) {
    Path path;

    double v_init = localization.speed * MPH_TO_MS;
    double s_init = localization.s;

    for (int i = 0; i < PATH_LENGTH; i++) {
      double t = (i + 1) * DELTA_T;
      double v = v_init + (0.4 * MAX_ACC * t);
      double s = (v_init + v) * t / 2;

      path.s.push_back(s_init + s);
      path.d.push_back(localization.d);
    }

    return path;
  }
};

class CruiseState : public State {
 public:
  Path build_path(const Localization localization) {
    Path path;
    double s_init = localization.s;
    double v = localization.speed * MPH_TO_MS;
    for (int i = 0; i < PATH_LENGTH; i++) {
      double t = (i + 1) * DELTA_T;
      double s = v * t;
      path.s.push_back(s_init + s);
      path.d.push_back(localization.d);
    }
    return path;
  }
};

// Brake is for emergency decceleration when other car cuts off
// Need to have jerk calculation for that
class BrakeState : public State {
};

// Decc is for maintaining the speed when approaching obstacle
class DeccState : public State {
 public:
  Path build_path(const Localization localization) {
    Path path;
    double v_init = localization.speed * MPH_TO_MS;
    double s_init = localization.s;

    for (int i = 0; i < PATH_LENGTH; i++) {
      double t = (i + 1) * DELTA_T;
      double v = v_init - (BRAKING_DECC * t);
      double delta_s = (v_init + v) * t / 2;
      double s = s_init + delta_s;
      if (s < s_init) {
        s = s_init;
      }

      path.s.push_back(s);
      path.d.push_back(localization.d);
    }
    return path;
  }
};

class StopState : public State {
 public:
  Path build_path(const Localization localization) {
    Path path;
    for (int i = 0; i < PATH_LENGTH; i++) {
      path.s.push_back(localization.s);
      path.d.push_back(localization.d);
    }
    return path;
  }
};

template<int direction>
class BaseSteeringState {
 public:
  /**
   * direction: -1 left, 0 straight, 1 right
   */
  Path build_path(const Localization localization, Path path_prev) {
    int lane_current = -1;
    if (0 <= localization.d && localization.d <= LANE_WIDTH) {
      lane_current = 0;
    } else if (LANE_WIDTH <= localization.d && localization.d < 2.0 * LANE_WIDTH) {
      lane_current = 1;
    } else if (2.0 * LANE_WIDTH <= localization.d && localization.d < 3.0 * LANE_WIDTH) {
      lane_current = 2;
    }

    // Move to the center of right lane
    double d_target = ((lane_current + direction) * LANE_WIDTH) + (LANE_WIDTH / 2);
    double distance_d = d_target - localization.d;

    std::vector<double> start_d{
        localization.d,
        path_prev.get_velocity_d(localization.d),
        0,
    };

    std::vector<double> end_d{
        (double)d_target,
        0,
        0,
    };

    std::vector<double> alphas_d = JMT(start_d, end_d, 2.0);

    Path path;
    for (int i = 0; i < PATH_LENGTH; i++) {
      double t = i * DELTA_T;
      double d = alphas_d[0] +
                 alphas_d[1] * t +
                 alphas_d[2] * pow(t, 2) +
                 alphas_d[3] * pow(t, 3) +
                 alphas_d[4] * pow(t, 4) +
                 alphas_d[5] * pow(t, 5);

      path.d.push_back(d);
    }

    json j;
    j["s"] = path.d;
    j["d"] = path.d;
    std::cout << "      jmt_s: " << j["s"].dump() << "\n";
    std::cout << "      jmt_d: " << j["d"].dump() << "\n";

    return path;
  }
};

class StraightState : public BaseSteeringState<0> {
};

class LeftState : public BaseSteeringState<-1> {
};

class RightState : public BaseSteeringState<1> {
};

#endif