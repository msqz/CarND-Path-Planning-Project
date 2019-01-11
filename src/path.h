#ifndef PATH_H
#define PATH_H

#include <vector>
#include "constraints.h"
#include "trajectory.h"

struct Path {
  std::vector<double> s;
  std::vector<double> d;
  Trajectory trajectory;

  int idx_latency;

  double get_velocity(double s) {
    double v = 0.0;
    for (int i = 1; i < this->size(); i++) {
      if (this->s[i - 1] <= s && s <= this->s[i]) {
        v = (this->s[i] - this->s[i - 1]) / DELTA_T;
      }
    }

    return v;
  }

  // only for cost calculations purpose
  double get_max_velocity() {
    double v_max = 0.0;
    for (int i = 1; i < 50; i++) {
      double v = (this->s[i] - this->s[i - 1]) / DELTA_T;
      if (v > v_max) {
        v_max = v;
      }
    }
    return v_max;
  }

  double get_velocity_d(double d) {
    double v = 0.0;
    for (int i = 1; i < this->size(); i++) {
      if (this->d[i - 1] <= d && d <= this->d[i] ||
          this->d[i - 1] >= d && d >= this->d[i]) {
        v = (this->d[i] - this->d[i - 1]) / DELTA_T;
        break;
      }
    }

    return v;
  }

  double get_acc_max_s() {
    double a_max = 0.0;
    for (int i = 2; i < this->size(); i++) {
      double v_0 = (this->s[i - 1] - this->s[i - 2]) / DELTA_T;
      double v_1 = (this->s[i] - this->s[i - 1]) / DELTA_T;
      double a = abs((v_1 - v_0)) / DELTA_T;
      if (a > a_max) {
        a_max = a;
      }
    }
    return a_max;
  }

  double get_acc_s(double s) {
    double acc = 0.0;
    for (int i = 2; i < this->size(); i++) {
      if (this->s[i - 1] <= s && s <= this->s[i]) {
        double v_0 = (this->s[i - 1] - this->s[i - 2]) / DELTA_T;
        double v_1 = (this->s[i] - this->s[i - 1]) / DELTA_T;
        acc = (v_1 - v_0) / DELTA_T;
        break;
      }
    }

    return acc;
  }

  double get_acc_d(double d) {
    double margin = 0.0;
    double acc = 0.0;
    for (int i = 2; i < this->size(); i++) {
      if (this->d[i - 1] - margin <= d && d <= this->d[i] + margin ||
          this->d[i - 1] - margin >= d && d >= this->d[i] + margin) {
        double v_0 = (this->d[i - 1] - this->d[i - 2]) / DELTA_T;
        double v_1 = (this->d[i] - this->d[i - 1]) / DELTA_T;
        acc = (v_1 - v_0) / DELTA_T;
        break;
      }
    }

    return acc;
  }

  bool contains_s(double s, double margin = 0) {
    if (this->s.front() - margin <= s && s <= this->s.back() + margin) {
      return true;
    }

    return false;
  }

  bool contains_d(double d, double margin = 0) {
    for (int i = 1; i < this->d.size(); i++) {
      if (this->d[i-1] - margin <= d && d <= this->d[i] + margin ||
          this->d[i-1] + margin >= d && d >= this->d[i] - margin) {
        return true;
      }
    }

    return false;
  }

  bool contains(double s, double d, double margin_s = 0.0, double margin_d = 0.0) {
    // TODO should I use theta_d here?

    // By using splitted continue/continue/return I can avoid 
    // redundant calculations of diff_d
    for (int i = 1; i < this->size(); i++) {
      double offset_s = CAR_LENGTH / 2 + margin_s / 2;
      double s_max = std::max(s, this->s[i]);
      double s_min = std::min(s, this->s[i]);

      double diff_s = (s_max - offset_s) - (s_min + offset_s);
      if (diff_s > 0) {
        continue;
      }

      double offset_d = CAR_WIDTH / 2 + margin_d / 2;
      double d_max = std::max(d, this->d[i]);
      double d_min = std::min(d, this->d[i]);
      double diff_d = (d_max - offset_d) - (d_min + offset_d);
      if (diff_d > 0) {
        continue;
      }

      return true;
    }

    return false;
  }

  int size() {
    return this->s.size();
  }
};

#endif
