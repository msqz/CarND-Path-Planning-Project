#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include <math.h>
#include <vector>
#include "constraints.h"
#include "localization.h"
#include "path.h"
#include "spline.h"
#include "trajectory.h"

const int WAYPOINTS_AHEAD = 20;
const int WAYPOINTS_BEHIND = 10;

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

  Trajectory generate(Path &path, Trajectory &trajectory_prev, Localization localization);
};

TrajectoryGenerator::TrajectoryGenerator(const Map &map) {
  this->map = map;
}

Trajectory TrajectoryGenerator::generate(Path &path, Trajectory &trajectory_prev, Localization localization) {
  Trajectory trajectory;
  std::vector<double> ptsx;
  std::vector<double> ptsy;

  double ref_x = localization.x;
  double ref_y = localization.y;
  double ref_yaw = localization.yaw * M_PI / 180.0;
  int prev_size = trajectory_prev.size();

  if (prev_size < 2) {
    double x_prev = ref_x - cos(ref_yaw);
    double y_prev = ref_y - sin(ref_yaw);
    ptsx.push_back(x_prev);
    ptsy.push_back(y_prev);

    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y);
  } else {
    double x_prev = trajectory_prev.x[prev_size - 2];
    double y_prev = trajectory_prev.y[prev_size - 2];
    ptsx.push_back(x_prev);
    ptsy.push_back(y_prev);

    ref_x = trajectory_prev.x[prev_size - 1];
    ref_y = trajectory_prev.y[prev_size - 1];
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y);

    ref_yaw = atan2(ref_y - y_prev, ref_x - x_prev);
  }

  std::vector<double> next_wp0 = this->map.get_xy(localization.s + HORIZON, path.d[path.size() - 1]);
  std::vector<double> next_wp1 = this->map.get_xy(localization.s + 2 * HORIZON, path.d[path.size() - 1]);
  std::vector<double> next_wp2 = this->map.get_xy(localization.s + 3 * HORIZON, path.d[path.size() - 1]);

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

  for (int i = 0; i < trajectory_prev.size(); i++) {
    trajectory.x.push_back(trajectory_prev.x[i]);
    trajectory.y.push_back(trajectory_prev.y[i]);
  }

  double target_x = HORIZON;  //horizon
  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

  double x_add_on = 0;

  for (int i = 1; i <= 50 - prev_size; i++) {
    double N = (target_dist / (0.02 * path.get_max_velocity()));
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    trajectory.x.push_back(x_point);
    trajectory.y.push_back(y_point);
  }

  return trajectory;
};

#endif