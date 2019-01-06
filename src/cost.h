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
  // std::cout << "    v: " << v_trajectory << "\n";

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
  double velocity_sum = 0.0;

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
    // If right now it's behind on the same lane then I don't care
    Localization loc_pred;
    loc_pred.d = prediction.d_original;
    if (loc_pred.get_lane() == localization.get_lane() && 
        prediction.s_original < localization.s) {
      // std::cout << "    ignoring " << prediction.id << "\n";
      continue;
    }

    // Check if there is something on the sides
    // First I need to find if the path is a curvy one
    if (std::abs(path.d.front() - path.d.back()) > 0.1) {
      double offset_s = CAR_LENGTH / 2;
      double s_max = std::max(localization.s, prediction.s_original);
      double s_min = std::min(localization.s, prediction.s_original);
      double diff_s = (s_max - offset_s) - (s_min + offset_s);
      if (diff_s <= 0) {
        double cost = 1 + abs(diff_s);
        if (cost > cost_max) {
          cost_max = cost;
        }
      }
    }

    // Check if path collide with any obstacle at it's predicted position or current position
    // TODO check if trajectory crosses the predicted straight path of the obstacle
    if (path.contains(prediction.s, prediction.d, 1.0, 0.5) ||
        path.contains(prediction.s_original, prediction.d_original, 1.0, 0.5)) {
      double distance = prediction.s - path.s.back();
      double distance_original = prediction.s_original - s_stop;
      if (distance < 0 || distance_original < 0) {
        // I want to handle scenario when all paths are going to hit
        // but the one with the largest distance can be selected.
        // For negative distance being closer means higher cost
        // (so at least it will hit with the minimal impact)
        double cost = 1 + abs(std::max(distance, distance_original));
        // There may be few obstacles colliding, need to evaluate against the nearest one
        // Otherwise if I evaluate against the far one it could hit the nearest one
        if (cost > cost_max) {
          std::cout << "{";
          std::cout << "\"id\": " << prediction.id;
          std::cout << ", \"s\": " << prediction.s;
          std::cout << ", \"d\": " << prediction.d;
          std::cout << ", \"dist\": " << distance;
          std::cout << ", \"dist_original\": " << distance_original;
          std::cout << "},"; 
          cost_max = cost;
        }
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
