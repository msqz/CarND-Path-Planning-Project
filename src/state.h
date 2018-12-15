#include <math.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "car.h"
#include "constraints.h"

#ifndef STATE_H
#define STATE_H

class State {
 public:
  virtual Trajectory build_trajectory(const Localization &localization) = 0;
};

class AccState : State {
 public:
  Trajectory build_trajectory(const Localization &localization) {
    Trajectory trajectory;

    double v_init = localization.speed * MPH_TO_MS;
    double s_init = localization.s;

    for (int i = 0; i < TRAJECTORY_LENGTH; i++) {
      double t = (i + 1) * DELTA_T;
      double v = v_init + MAX_ACC * t;
      double s = (v_init + v) * t / 2;

      trajectory.s.push_back(s_init + s);
      trajectory.d.push_back(localization.d);
    }

    return trajectory;
  }
};

class CruiseState : State {
 public:
  Trajectory build_trajectory(const Localization &localization) {
    Trajectory trajectory;
    double s_init = localization.s;
    double v = localization.speed * MPH_TO_MS;
    for (int i = 0; i < TRAJECTORY_LENGTH; i++) {
      double t = (i + 1) * DELTA_T;
      double s = v * t;
      trajectory.s.push_back(s_init + s);
      trajectory.d.push_back(localization.d);
    }
    return trajectory;
  }
};

class DeccState : State {
 public:
  Trajectory build_trajectory(const Localization &localization) {
    Trajectory trajectory;
    double v_init = localization.speed * MPH_TO_MS;
    double s_init = localization.s;

    for (int i = 0; i < TRAJECTORY_LENGTH; i++) {
      double t = (i + 1) * DELTA_T;
      double v = v_init - MAX_ACC * t;
      double s = (v_init + v) * t / 2;

      trajectory.s.push_back(s_init + s);
      trajectory.d.push_back(localization.d);
    }
    return trajectory;
  }
};

class StopState : State {
 public:
  Trajectory build_trajectory(const Localization &localization) {
    Trajectory trajectory;
    trajectory.s.push_back(localization.s);
    trajectory.d.push_back(localization.d);
    return trajectory;
  }
};

#endif