#include <math.h>
#include <iostream>
#include "car.h"

#ifndef STATE_H
#define STATE_H

const double MAX_ACC = 10;
const double DELTA_T = 0.2;
const double SPEED_LIMIT = 50;
const double BUFFER_V = 5;
const double MPH_TO_MS = 0.447;

class State {
 public:
  virtual Trajectory build_trajectory(const Localization &localization) = 0;
};

class AccState : public State {
 public:
  Trajectory build_trajectory(const Localization &localization) {
    Trajectory trajectory;

    double v_init = localization.speed * MPH_TO_MS;
    double s_init = localization.s;

    for (int i = 0; i < 50; i++) {
      double t = (i + 1) * DELTA_T;
      double v = v_init + MAX_ACC * t;
      double s = (v_init + v) * t / 2;

      trajectory.s.push_back(s_init + s);
      trajectory.d.push_back(localization.d);
      // std::cout << i << " : " << (v_init + MAX_ACC * t) << " : " << (s_init + s) << "\n";
    }

    return trajectory;
  }
};

class CruiseState : public State {
 public:
  Trajectory build_trajectory(const Localization &localization) {
    Trajectory trajectory;
    double s_init = localization.s;
    double v = localization.speed * MPH_TO_MS || 1;
    for (int i = 0; i < 10; i++) {
      double t = (i + 1) * DELTA_T;
      double s = v * t;
      trajectory.s.push_back(s_init + s);
      trajectory.d.push_back(localization.d);
      // std::cout<<"s: " <<s<<"\n";
    }
    return trajectory;
  }
};

#endif