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

  int size() {
    return s.size();
  }
};

class TrajectoryGenerator {
 private:
  Map map;

  std::tuple<Waypoint, Waypoint> get_nearest_waypoints(double s) {
    int n_waypoints = this->map.size();
    int wp_ahead;
    for (int i = 0; i < n_waypoints; i++) {
      if (this->map.s[i] >= s) {
        wp_ahead = i;
        break;
      }
    }
    int wp_behind = wp_ahead - 1;
    if (wp_behind < 0) {
      wp_behind = (n_waypoints + wp_behind);
    }

    Waypoint wp_1 = {
        .x = this->map.x[wp_behind],
        .y = this->map.y[wp_behind],
        .dx = this->map.dx[wp_behind],
        .dy = this->map.dy[wp_behind],
    };
    Waypoint wp_2 = {
        .x = this->map.x[wp_ahead],
        .y = this->map.y[wp_ahead],
        .dx = this->map.dx[wp_ahead],
        .dy = this->map.dy[wp_ahead],
    };

    return {wp_1, wp_2};
  }

  std::vector<double> get_xy2(double s, double d) {
    int prev_wp = -1;

    while (s > this->map.s[prev_wp + 1] && (prev_wp < (int)(this->map.s.size() - 1))) {
      prev_wp++;
    }

    int wp2 = (prev_wp + 1) % this->map.x.size();

    double heading = atan2((this->map.y[wp2] - this->map.y[prev_wp]), (this->map.x[wp2] - this->map.x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - this->map.s[prev_wp]);

    double seg_x = this->map.x[prev_wp] + seg_s * cos(heading);
    double seg_y = this->map.y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - M_PI / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);
    

    return {x, y};
  }

  std::vector<tk::spline> get_splines(double s, double d) {
    int wp_30_idx;
    int wp_60_idx;
    int wp_90_idx;
    for (int i = 1; i < this->map.size(); i++) {
      if (this->map.s[i - 1] <= s + 30 && s + 30 <= this->map.s[i]) {
        wp_30_idx = i;
      } else if (this->map.s[i - 1] <= s + 60 && s + 60 <= this->map.s[i]) {
        wp_60_idx = i;
      } else if (this->map.s[i - 1] <= s + 90 && s + 90 <= this->map.s[i]) {
        wp_90_idx = i;
      }
    }

    std::vector<double> s_values{this->map.s[wp_30_idx], this->map.s[wp_60_idx], this->map.s[wp_90_idx]};
    std::vector<double> x_values{this->map.x[wp_30_idx], this->map.x[wp_60_idx], this->map.x[wp_90_idx]};
    std::vector<double> y_values{this->map.y[wp_30_idx], this->map.y[wp_60_idx], this->map.y[wp_90_idx]};
    std::vector<double> dx_values{this->map.dx[wp_30_idx], this->map.dx[wp_60_idx], this->map.dx[wp_90_idx]};
    std::vector<double> dy_values{this->map.dy[wp_30_idx], this->map.dy[wp_60_idx], this->map.dy[wp_90_idx]};

    tk::spline spline_s_x;
    spline_s_x.set_points(s_values, x_values);

    tk::spline spline_s_y;
    spline_s_y.set_points(s_values, y_values);

    tk::spline spline_s_dx;
    spline_s_dx.set_points(s_values, dx_values);

    tk::spline spline_s_dy;
    spline_s_dy.set_points(s_values, dy_values);

    return {spline_s_x, spline_s_y, spline_s_dx, spline_s_dy};
  }

  std::vector<double> get_xy(double s, double d) {
    int n_waypoints = this->map.size();
    int next_wp;
    for (int i = 0; i < n_waypoints; i++) {
      if (this->map.s[i] >= s) {
        next_wp = i;
        break;
      }
    }

    int wp_behind = next_wp;
    for (int i = 0; i < WAYPOINTS_BEHIND; i++) {
      wp_behind = wp_behind - 1;
      // Handle leap to the end of the track
      if (wp_behind < 0) {
        wp_behind = (n_waypoints + wp_behind);
      }
      wp_behind = wp_behind % n_waypoints;
    }

    int wp_ahead = next_wp;
    for (int i = 0; i < WAYPOINTS_AHEAD; i++) {
      wp_ahead = (wp_ahead + 1) % n_waypoints;
    }

    std::vector<double> s_values;
    std::vector<double> x_values;
    std::vector<double> y_values;
    std::vector<double> dx_values;
    std::vector<double> dy_values;

    // Handling the track leap (going from waypoint 180 to 0)
    int idx_start = wp_behind;
    int idx_end = wp_ahead;
    if (idx_start > idx_end) {
      idx_end += n_waypoints + 1;
    }

    while (idx_start < idx_end) {
      int idx = idx_start;
      if (idx > n_waypoints) {
        idx -= n_waypoints;
      }

      double s_value = this->map.s[idx];
      if (s_value > this->map.s[wp_ahead]) {
        s_value = s_value - TRACK_LENGTH;
      }
      s_values.push_back(s_value);
      x_values.push_back(this->map.x[idx]);
      y_values.push_back(this->map.y[idx]);
      dx_values.push_back(this->map.dx[idx]);
      dy_values.push_back(this->map.dy[idx]);

      ++idx_start;
    }

    // TODO Handle end of the track:
    // Assertion `m_x[i]<m_x[i+1]' failed
    tk::spline spline_s_x;
    spline_s_x.set_points(s_values, x_values);

    tk::spline spline_s_y;
    spline_s_y.set_points(s_values, y_values);

    tk::spline spline_s_dx;
    spline_s_dx.set_points(s_values, dx_values);

    tk::spline spline_s_dy;
    spline_s_dy.set_points(s_values, dy_values);

    double x = spline_s_x(s) + spline_s_dx(s) * d;
    double y = spline_s_y(s) + spline_s_dy(s) * d;

    return {x, y};
  }

 public:
  TrajectoryGenerator(const Map &map);

  Trajectory generate(const Path &path, Trajectory &trajectory_prev, double end_path_s, double x_last, double y_last);
};

TrajectoryGenerator::TrajectoryGenerator(const Map &map) {
  this->map = map;
}

Trajectory TrajectoryGenerator::generate(const Path &path, Trajectory &trajectory_prev, double end_path_s, double x_last, double y_last) {
  Trajectory trajectory;

  for (int i = 0; i < path.s.size(); i++) {
    std::vector<double> xy = this->get_xy(path.s[i], path.d[i]);

    trajectory.x.push_back(xy[0]);
    trajectory.y.push_back(xy[1]);
    trajectory.waypoints.push_back(this->get_nearest_waypoints(path.s[i]));
  }

  // std::vector<tk::spline> splines = get_splines(path.s[0], path.d[0]);

  // for (int i = 0; i < path.s.size(); i++) {
  //   double x = splines[0](path.s[i]) + splines[2](path.s[i]) * path.d[i];
  //   double y = splines[1](path.s[i]) + splines[3](path.s[i]) * path.d[i];

  //   trajectory.x.push_back(x);
  //   trajectory.y.push_back(y);
  //   trajectory.waypoints.push_back(this->get_nearest_waypoints(path.s[i]));
  // }


  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  if (trajectory_prev.size() > 9990) {
    // next_x_vals.push_back(x_last);
    // next_y_vals.push_back(y_last);
    next_x_vals.push_back(trajectory_prev.x[0]);
    next_y_vals.push_back(trajectory_prev.y[0]);
  } else {
    next_x_vals.push_back(trajectory.x[0]);
    next_y_vals.push_back(trajectory.y[0]);
  }

  for (int i = 1; i < trajectory.size(); i++) {
    double delta_x = trajectory.x[i] - trajectory.x[i - 1];
    double delta_y = trajectory.y[i] - trajectory.y[i - 1];

    next_x_vals.push_back(next_x_vals[i - 1] + delta_x);
    next_y_vals.push_back(next_y_vals[i - 1] + delta_y);
  }

  trajectory.x = next_x_vals;
  trajectory.y = next_y_vals;

  // double s = end_path_s;
  // // double s = path.s[0];
  // trajectory.waypoints.push_back(this->get_nearest_waypoints(s));
  // for (int i = 1; i < path.s.size(); i++) {
  //   double delta_s = path.s[i] - path.s[i-1];
  //   s += delta_s;
  //   trajectory.waypoints.push_back(this->get_nearest_waypoints(s));
  // }

  return trajectory;
};

#endif