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
    {"DECC", {"CRUISE", "STOP", "DECC", "ACC"}},
    {"CRUISE", {"ACC", "DECC", "CRUISE"}},
};

class State {
 public:
  virtual Path build_path(const Localization &localization){};
};

class AccState : public State {
 public:
  Path build_path(const Localization &localization) {
    Path path;

    double v_init = localization.speed * MPH_TO_MS;
    double s_init = localization.s;

    for (int i = 0; i < PATH_LENGTH; i++) {
      double t = (i + 1) * DELTA_T;
      double v = v_init + (0.9 * MAX_ACC * t);
      double s = (v_init + v) * t / 2;

      path.s.push_back(s_init + s);
      path.d.push_back(localization.d);
    }

    return path;
  }
};

class CruiseState : public State {
 public:
  Path build_path(const Localization &localization) {
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

class DeccState : public State {
 public:
  Path build_path(const Localization &localization) {
    Path path;
    double v_init = localization.speed * MPH_TO_MS;
    double s_init = localization.s;

    for (int i = 0; i < PATH_LENGTH; i++) {
      double t = (i + 1) * DELTA_T;
      double v = v_init - (0.9 * MAX_ACC * t);
      double s = (v_init + v) * t / 2;

      path.s.push_back(s_init + s);
      path.d.push_back(localization.d);
    }
    return path;
  }
};

class StopState : public State {
 public:
  Path build_path(const Localization &localization) {
    Path path;
    path.s.push_back(localization.s);
    path.d.push_back(localization.d);
    return path;
  }
};

#endif