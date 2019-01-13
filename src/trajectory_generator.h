#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include <math.h>
#include <vector>
#include "constraints.h"
#include "localization.h"
#include "path.h"
#include "spline.h"
#include "trajectory.h"

const int WAYPOINTS_AHEAD = 10;
const int WAYPOINTS_BEHIND = 5;

struct Map {
  std::vector<double> s;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> dx;
  std::vector<double> dy;

  tk::spline spline_s_x;
  tk::spline spline_s_y;
  tk::spline spline_s_dx;
  tk::spline spline_s_dy;

  void init() {
    this->spline_s_x.set_points(this->s, this->x);
    this->spline_s_y.set_points(this->s, this->y);
    this->spline_s_dx.set_points(this->s, this->dx);
    this->spline_s_dy.set_points(this->s, this->dy);
  }

  std::vector<double> get_xy(double s, double d) {
    double x = this->spline_s_x(s) + this->spline_s_dx(s) * d;
    double y = this->spline_s_y(s) + this->spline_s_dy(s) * d;

    return {x, y};
  }

  int size() {
    return s.size();
  }
};

class TrajectoryGenerator {
 private:
  Map map;

 public:
  TrajectoryGenerator(const Map &map);

  Trajectory generate(const Path &path, Trajectory &trajectory_prev, Localization localization);
};

TrajectoryGenerator::TrajectoryGenerator(const Map &map) {
  this->map = map;
}

Trajectory TrajectoryGenerator::generate(const Path &path, Trajectory &trajectory_prev, Localization localization) {
  Trajectory trajectory;
  for (int i = 0; i < path.s.size(); i++) {
    std::vector<double> xy = this->map.get_xy(path.s[i], path.d[i]);
    trajectory.x.push_back(xy[0]);
    trajectory.y.push_back(xy[1]);
  }

  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  int usage_prev = 5;
  if (trajectory_prev.size() > usage_prev) {
    for (int i=0; i < usage_prev; i++) {
      next_x_vals.push_back(trajectory_prev.x[i]);
      next_y_vals.push_back(trajectory_prev.y[i]);
    }
  } else {
    next_x_vals.push_back(trajectory.x[0]);
    next_y_vals.push_back(trajectory.y[0]);
  }

  double next_idx = next_x_vals.size() - 1;
  for (int i = 1; i < trajectory.size(); i++) {
    double delta_x = trajectory.x[i] - trajectory.x[i - 1];
    double delta_y = trajectory.y[i] - trajectory.y[i - 1];

    next_x_vals.push_back(next_x_vals[next_idx + i - 1] + delta_x);
    next_y_vals.push_back(next_y_vals[next_idx + i - 1] + delta_y);
  }

  trajectory.x = next_x_vals;
  trajectory.y = next_y_vals;

  return trajectory;
};

#endif