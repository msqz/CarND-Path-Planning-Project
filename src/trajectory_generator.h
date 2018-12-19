#include <vector>
#include "path.h"
#include "trajectory.h"
#include "spline.h"
#include "constraints.h"
#include "localization.h"

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

  std::vector<double> get_xy(double s, double d) {
    int next_wp;
    for (int i = 0; i < this->map.size(); i++) {
      if (this->map.s[i] >= s) {
        next_wp = i;
        break;
      }
    }


    int waypoints_no = this->map.size();
    int wp_behind = next_wp;
    for (int i = 0; i < 5; i++) {
      wp_behind = wp_behind - 1;
      // Handle leap to the end of the track
      if (wp_behind < 0) {
        wp_behind = (waypoints_no + wp_behind);
      }
      wp_behind = wp_behind % waypoints_no;
    }

    int wp_ahead = next_wp;
    for (int i = 0; i < 10; i++) {
      wp_ahead = (wp_ahead + 1) % waypoints_no;
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
      idx_end += this->map.size() + 1;
    }
    while (idx_start < idx_end) {
      int idx = idx_start;
      if (idx > this->map.size()) {
        idx -= this->map.size();
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

  Trajectory generate(const Path &path);
};

TrajectoryGenerator::TrajectoryGenerator(const Map &map) {
  this->map = map;
}

Trajectory TrajectoryGenerator::generate(const Path &path){
  Trajectory trajectory;
  for (int i = 0; i < path.s.size(); i++) {
    std::vector<double> xy = this->get_xy(path.s[i], path.d[i]);
    trajectory.x.push_back(xy[0]);
    trajectory.y.push_back(xy[1]);
  }

  return trajectory;
};