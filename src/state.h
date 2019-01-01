#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include "localization.h"
#include "obstacle.h"
#include "path.h"

std::map<std::string, std::vector<std::string>> STATES = {
    {"KL", {"KL"}}
    // {"KL", {"KL", "LCL"}},
    // {"LCL", {"KL", "LCL"}},
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

  return alphas;
}

class State {
 protected:
  virtual std::vector<std::vector<double>> get_cruise_parameters(Localization loc, Path path_prev, std::vector<Obstacle> obst) = 0;
  virtual std::vector<std::vector<double>> get_acc_parameters(Localization loc, Path path_prev, std::vector<Obstacle> obst) = 0;
  virtual std::vector<std::vector<double>> get_decc_parameters(Localization loc, Path path_prev, std::vector<Obstacle> obst) = 0;

  Path build_path(std::vector<double> s_i, std::vector<double> s_f, std::vector<double> d_i, std::vector<double> d_f) {
    Path path;

    std::vector<double> coeff_s = JMT(s_i, s_f, DELTA_T * PATH_LENGTH);
    for (int i = 0; i < PATH_LENGTH; i++) {
      double t = i * DELTA_T;
      double s = coeff_s[0] +
                 coeff_s[1] * t +
                 coeff_s[2] * pow(t, 2) +
                 coeff_s[3] * pow(t, 3) +
                 coeff_s[4] * pow(t, 4) +
                 coeff_s[5] * pow(t, 5);

      path.s.push_back(s);
    };

    std::vector<double> coeff_d = JMT(d_i, d_f, DELTA_T * PATH_LENGTH);
    for (int i = 0; i < PATH_LENGTH; i++) {
      double t = i * DELTA_T;
      double d = coeff_d[0] +
                 coeff_d[1] * t +
                 coeff_d[2] * pow(t, 2) +
                 coeff_d[3] * pow(t, 3) +
                 coeff_d[4] * pow(t, 4) +
                 coeff_d[5] * pow(t, 5);

      path.d.push_back(d);
    };

    return path;
  };

 public:
  std::vector<Path> build_paths(Localization loc, Path path_prev, std::vector<Obstacle> obst) {
    std::vector<std::vector<double>> parameters_cruise = this->get_cruise_parameters(loc, path_prev, obst);
    Path path_cruise = this->build_path(parameters_cruise[0], parameters_cruise[1], parameters_cruise[2], parameters_cruise[3]);
    path_cruise.label = "ACC";

    std::vector<std::vector<double>> parameters_acc = this->get_acc_parameters(loc, path_prev, obst);
    Path path_acc = this->build_path(parameters_acc[0], parameters_acc[1], parameters_acc[2], parameters_acc[3]);
    path_acc.label = "CRUISE";

    std::vector<std::vector<double>> parameters_decc = this->get_decc_parameters(loc, path_prev, obst);
    Path path_decc = this->build_path(parameters_decc[0], parameters_decc[1], parameters_decc[2], parameters_decc[3]);
    path_decc.label = "DECC";

    return {
        path_cruise,
        path_acc,
        path_decc,
    };
  };
};

class KL : public State {
 protected:
  std::vector<std::vector<double>> get_cruise_parameters(Localization loc, Path path_prev, std::vector<Obstacle> obst) {
    double v = loc.speed * MPH_TO_MS;
    double a = path_prev.get_acc_s(loc.s);
    std::vector<double> s_i{loc.s, v, a};
    std::vector<double> s_f{loc.s + (PATH_LENGTH * DELTA_T * v), v, 0};
    std::vector<double> d_i{loc.d, 0, 0};
    std::vector<double> d_f{loc.d, 0, 0};

    return {s_i, s_f, d_i, d_f};
  };

  std::vector<std::vector<double>> get_acc_parameters(Localization loc, Path path_prev, std::vector<Obstacle> obst) {
    double v = loc.speed * MPH_TO_MS;
    double a = path_prev.get_acc_s(loc.s);
    std::vector<double> s_i{loc.s, v, a};

    a = MAX_ACC;
    v = v + (0.4 * a);
    std::vector<double> s_f{loc.s + (PATH_LENGTH * DELTA_T * v), v, a};

    std::vector<double> d_i{loc.d, 0, 0};
    std::vector<double> d_f{loc.d, 0, 0};

    return {s_i, s_f, d_i, d_f};
  };

  std::vector<std::vector<double>> get_decc_parameters(Localization loc, Path path_prev, std::vector<Obstacle> obst) {
    double v = loc.speed * MPH_TO_MS;
    double a = path_prev.get_acc_s(loc.s);
    std::vector<double> s_i{loc.s, v, a};

    a = BRAKING_DECC;
    v = v - (0.7 * a);

    double s = loc.s + PATH_LENGTH * DELTA_T * v;
    if (s < loc.s) {
      s = loc.s;
    }
    std::vector<double> s_f{s, v, a};

    std::vector<double> d_i{loc.d, 0, 0};
    std::vector<double> d_f{loc.d, 0, 0};

    return {s_i, s_f, d_i, d_f};
  };
};

class LCL : public State {
 protected:
  std::vector<std::vector<double>> get_cruise_parameters(Localization loc, Path path_prev, std::vector<Obstacle> obst) {
    double v = loc.speed * MPH_TO_MS;
    double a = path_prev.get_acc_s(loc.s);
    std::vector<double> s_i{loc.s, v, a};
    std::vector<double> s_f{loc.s + (PATH_LENGTH * DELTA_T * v), v, 0};
    std::vector<double> d_i{loc.d, 0, 0};

    double d = (loc.get_lane() - 1) + 2.0;
    std::vector<double> d_f{d, 0, 0};

    return {s_i, s_f, d_i, d_f};
  };

  std::vector<std::vector<double>> get_acc_parameters(Localization loc, Path path_prev, std::vector<Obstacle> obst) {
    double v = loc.speed * MPH_TO_MS;
    double a = path_prev.get_acc_s(loc.s);
    std::vector<double> s_i{loc.s, v, a};

    a = MAX_ACC;
    v = v + (0.4 * a);
    std::vector<double> s_f{loc.s + (PATH_LENGTH * DELTA_T * v), v, a};

    std::vector<double> d_i{loc.d, 0, 0};
    // TODO calculate final d
    double d = (loc.get_lane() - 1) + 2.0;
    std::vector<double> d_f{d, 0, 0};

    //TODO calculate it once per cycle in Predictions
    // bool target_found = false;
    // Obstacle target;
    // double dist_min = std::numeric_limits<double>::infinity();
    // for (Obstacle &o : obst) {
    //   if (o.get_lane() == loc.get_lane() && o.s > loc.s) {
    //     double dist = o.s - loc.s;
    //     if (dist < dist_min) {
    //       target = o;
    //       dist_min = dist;
    //       target_found = true;
    //     }
    //   }
    // }

    // if (target_found) {
    //   s_f[0] = target.s;
    // }

    return {s_i, s_f, d_i, d_f};
  };

  std::vector<std::vector<double>> get_decc_parameters(Localization loc, Path path_prev, std::vector<Obstacle> obst) {
    double v = loc.speed * MPH_TO_MS;
    double a = path_prev.get_acc_s(loc.s);
    std::vector<double> s_i{loc.s, v, a};

    a = BRAKING_DECC;
    v = v - (0.7 * a);

    double s = loc.s + PATH_LENGTH * DELTA_T * v;
    if (s < loc.s) {
      s = loc.s;
    }
    std::vector<double> s_f{s, v, a};

    std::vector<double> d_i{loc.d, 0, 0};

    double d = (loc.get_lane() - 1) + 2.0;
    std::vector<double> d_f{d, 0, 0};

    return {s_i, s_f, d_i, d_f};
  };
};

class LCR : public State {
};