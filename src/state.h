#ifndef STATE_H
#define STATE_H

#include <math.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "constraints.h"
#include "localization.h"
#include "path.h"

std::map<std::string, std::vector<std::string>> STATES = {
    {"STOP", {"ACC", "STOP"}},
    {"ACC", {"CRUISE", "ACC", "DECC"}},
    {"DECC", {"CRUISE", "DECC", "ACC", "STOP"}},
    {"CRUISE", {"ACC", "DECC", "CRUISE"}},
};

std::map<std::string, std::vector<std::string>> STATES_D = {
    {"STRAIGHT", {"STRAIGHT"}},
};

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

class StraightState : public State {
 public:
  Path build_path(const Localization localization) {
    Path path;
    for (int i = 0; i < PATH_LENGTH; i++) {
      path.d.push_back(localization.d);
    }
    return path;
  }
};

class LeftState : public CruiseState {
};

class RightState : public CruiseState {
 public:
  Path build_path(const Localization localization) {
    Path path = CruiseState::build_path(localization);
    int lane_current = localization.d / LANE_WIDTH;
    // Move to the center of right lane

    int d_target = ((lane_current + 1) * LANE_WIDTH) + (LANE_WIDTH / 2);
    double distance_d = d_target - localization.d;
    for (int i = 0; i < PATH_LENGTH; i++) {
      path.d[i] = localization.d + distance_d;
    }
    // std::cout << "    d: " << localization.d << "\n";
    // std::cout << "    d_path: " << path.d.back() << "\n";

    return path;
  }
};

#endif