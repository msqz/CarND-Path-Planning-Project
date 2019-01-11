#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

struct Waypoint {
  double x;
  double y;
  double dx;
  double dy;
};

struct Trajectory {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<std::tuple<Waypoint, Waypoint>> waypoints;

  int size() {
    return x.size();
  }
};
#endif