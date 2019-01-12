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
#include "obstacle.h"
#include "path.h"

using json = nlohmann::json;

std::map<std::string, std::vector<std::string>> STATES_S = {
    {"STOP", {"ACC_SOFT", "ACC_MED", "ACC_HARD", "STOP"}},
    {"ACC_SOFT", {"CRUISE", "ACC_SOFT", "ACC_MED", "ACC_HARD", "DECC_SOFT", "DECC_MED", "DECC_HARD"}},
    {"ACC_MED", {"CRUISE", "ACC_SOFT", "ACC_MED", "ACC_HARD", "DECC_SOFT", "DECC_MED", "DECC_HARD"}},
    {"ACC_HARD", {"CRUISE", "ACC_SOFT", "ACC_MED", "ACC_HARD", "DECC_SOFT", "DECC_MED", "DECC_HARD"}},
    {"DECC_SOFT", {"CRUISE", "DECC_SOFT", "DECC_MED", "DECC_HARD", "ACC_SOFT", "ACC_MED", "ACC_HARD"}},
    {"DECC_MED", {"CRUISE", "DECC_SOFT", "DECC_MED", "DECC_HARD", "ACC_SOFT", "ACC_MED", "ACC_HARD"}},
    {"DECC_HARD", {"CRUISE", "DECC_SOFT", "DECC_MED", "DECC_HARD", "ACC_SOFT", "ACC_MED", "ACC_HARD"}},
    {"CRUISE", {"ACC_SOFT", "ACC_MED", "ACC_HARD", "DECC_SOFT", "DECC_MED", "DECC_HARD", "CRUISE"}},
    // {"STOP", {"ACC_SOFT", "ACC_MED", "ACC_HARD"}},
    // {"ACC_SOFT", {"CRUISE", "ACC_SOFT", "ACC_MED", "ACC_HARD", "DECC_HARD"}},
    // {"ACC_MED", {"CRUISE", "ACC_SOFT", "ACC_MED", "ACC_HARD", "DECC_HARD"}},
    // {"ACC_HARD", {"CRUISE", "ACC_SOFT", "ACC_MED", "ACC_HARD", "DECC_HARD"}},
    // {"DECC_HARD", {"CRUISE", "DECC_HARD", "ACC_SOFT", "ACC_MED", "ACC_HARD"}},
    // {"CRUISE", {"ACC_SOFT", "ACC_MED", "ACC_HARD", "DECC_HARD", "CRUISE"}},
};

std::map<std::string, std::vector<std::string>> STATES_D = {
    // {"STRAIGHT", {"STRAIGHT", "RIGHT", "LEFT", "DOUBLE_LEFT", "DOUBLE_RIGHT"}},
    // {"RIGHT", {"STRAIGHT", "RIGHT", "DOUBLE_RIGHT"}},
    // {"LEFT", {"STRAIGHT", "LEFT", "DOUBLE_LEFT"}},
    // {"DOUBLE_RIGHT", {"STRAIGHT", "RIGHT", "DOUBLE_RIGHT"}},
    // {"DOUBLE_LEFT", {"STRAIGHT", "LEFT", "DOUBLE_LEFT"}},
    {"STRAIGHT", {"STRAIGHT", "RIGHT", "LEFT"}},
    {"RIGHT", {"STRAIGHT", "RIGHT"}},
    {"LEFT", {"STRAIGHT", "LEFT"}},
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

  std::vector<double> points;
  for (int i = 0; i < PATH_LENGTH; i++) {
    double dt = i * DELTA_T;
    double p = alphas[0] +
               alphas[1] * dt +
               alphas[2] * pow(dt, 2) +
               alphas[3] * pow(dt, 3) +
               alphas[4] * pow(dt, 4) +
               alphas[5] * pow(dt, 5);

    points.push_back(p);
  }

  return points;
}

class State {
 public:
  virtual Path build_path(const Localization localization, Path path_prev) = 0;
};

class AccState {
 public:
  Path build_path(const Localization localization, Path path_prev, double a) {
    Path path;

    double t = DELTA_T * PATH_LENGTH;

    double s_i = localization.s;
    double s_dot_i = localization.speed * MPH_TO_MS;
    double s_ddot_i = path_prev.get_acc_s(s_i);

    double s_f = s_i + (s_dot_i * t) + (a * pow(t, 2) / 2);
    double s_dot_f = s_dot_i + (a * t);
    double s_ddot_f = 0;

    path.s = JMT({s_i, s_dot_i, s_ddot_i}, {s_f, s_dot_f, s_ddot_f}, t);

    return path;
  }
};

class CruiseState : public State {
 public:
  Path build_path(const Localization localization, Path path_prev) {
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

class DeccState {
 public:
  Path build_path(const Localization localization, Path path_prev, double a) {
    Path path;
    double v_init = localization.speed * MPH_TO_MS;
    double s_init = localization.s;

    for (int i = 0; i < PATH_LENGTH; i++) {
      double t = (i + 1) * DELTA_T;
      double v = v_init - (a * t);
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
  Path build_path(const Localization localization, Path path_prev) {
    Path path;
    for (int i = 0; i < PATH_LENGTH; i++) {
      path.s.push_back(localization.s);
      path.d.push_back(localization.d);
    }
    return path;
  }
};

template <int direction>
class BaseSteeringState {
 public:
  /**
   * direction: -1 left, 0 straight, 1 right
   */
  Path build_path(Localization localization, Path path_prev) {
    int lane_current = localization.get_lane();

    // Move to the center of the lane
    double d_target = ((lane_current + direction) * LANE_WIDTH) + (LANE_WIDTH / 2);

    std::vector<double> start_d{
        localization.d,
        path_prev.get_velocity_d(localization.d),
        path_prev.get_acc_d(localization.d),
    };

    std::vector<double> end_d{
        (double)d_target,
        0,
        0,
    };

    double t = PATH_LENGTH * DELTA_T;
    Path path;
    path.d = JMT(start_d, end_d, t);

    return path;
  }
};

class StraightState : public BaseSteeringState<0> {
};

class LeftState : public BaseSteeringState<-1> {
};

class RightState : public BaseSteeringState<1> {
};

class DoubleLeftState : public BaseSteeringState<-2> {
};

class DoubleRightState : public BaseSteeringState<2> {
};

#endif
