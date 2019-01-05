#ifndef COST_H
#define COST_H

#include <math.h>
#include <iostream>
#include "constraints.h"
#include "json.hpp"
#include "localization.h"
#include "obstacle.h"
#include "path.h"
#include "tracking.h"

double sigmoid(double x) {
  return 1 / (1 + exp(-x));
}

double evaluate_speed_limit(Path trajectory) {
  double v_target = SPEED_LIMIT - BUFFER_V;
  double v_trajectory = trajectory.get_max_velocity() * MS_TO_MPH;
  std::cout << "    v: " << v_trajectory << "\n";

  if (v_trajectory > SPEED_LIMIT) {
    return 1;
  }
  return 0;
}

double evaluate_max_acc(Path path, Localization localization) {
  if (path.get_max_acc() > MAX_ACC) {
    return 1;
  }

  return 0;
}

double evaluate_efficiency(Path trajectory) {
  // Average speed
  double velocity_sum;

  for (int i = 1; i < trajectory.s.size(); i++) {
    velocity_sum += (trajectory.s[i] - trajectory.s[i - 1]) / DELTA_T;
  }

  if (velocity_sum == 0) {
    return 1;
  }

  double velocity_avg = velocity_sum / (double)trajectory.size();
  if (velocity_avg <= 0) {
    return 1;
  }

  return 1 / velocity_avg;
}

double evaluate_crash(Path path, Localization localization, std::vector<Prediction> predictions) {
  //how many meters do I need to stop (v = 0)?
  double v_max = path.get_max_velocity();
  double t_stop = v_max / BRAKING_DECC;
  // braking distance + buffer distance
  double s_stop = localization.s +
                  ((BRAKING_DECC * (t_stop * t_stop)) / 2) +
                  FRONT_DISTANCE;

  double cost_max = 0.0;
  double dt = PATH_LENGTH * DELTA_T;
  for (const Prediction &prediction : predictions) {
    // Check if there's a car around
    // TODO it might be redundant
    // std::tuple<double, double> corner_lt = {localization.s + CAR_LENGTH,
    //                                         localization.d - CAR_WIDTH};
    // std::tuple<double, double> corner_rt = {localization.s + CAR_LENGTH,
    //                                         localization.d - CAR_WIDTH};
    // std::tuple<double, double> corner_rb = {localization.s + CAR_LENGTH,
    //                                         localization.d + CAR_WIDTH};
    // std::tuple<double, double> corner_lb = {localization.s - CAR_LENGTH,
    //                                         localization.d - CAR_WIDTH};
    // Check if path collide with any obstacle
    if (path.contains(prediction.s, prediction.d, 1.0, 0.5)) {
      double distance = prediction.s - s_stop;
      if (distance < 0) {
        // I want to handle scenario when all paths are going to hit
        // but the one with the largest distance can be selected.
        // For negative distance being closer means higher cost
        // (so at least it will hit with the minimal impact)
        std::cout << "    id: " << prediction.id
                  << ", s: " << prediction.s
                  << ", d: " << prediction.d
                  << ", dist: " << distance << "\n";
        cost_max = 1 + abs(distance);
      }
    }
  }
  return cost_max;
}

double evaluate_offroad(Path path) {
  for (const double &d : path.d) {
    if (d < 1.5 || d > 11.0) {
      return 1;
    }
  }
  return 0;
}

double evaluate_keep_right(Path path) {
  if (path.d.back() < 9.0) {
    return (10.0 - path.d.back()) / 10.0;
  }
  return 0;
}

#endif
