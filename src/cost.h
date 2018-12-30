#ifndef COST_H
#define COST_H

#include <math.h>
#include <iostream>
#include "constraints.h"
#include "json.hpp"
#include "localization.h"
#include "obstacle.h"
#include "path.h"

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

double evaluate_crash(Path path, Localization localization, std::vector<Obstacle> obstacles) {
  //how many meters do I need to stop (v = 0)?
  double v_max = path.get_max_velocity();
  double t_stop = v_max / BRAKING_DECC;
  // braking distance + buffer distance
  double s_stop = path.s.front() +
                  ((BRAKING_DECC * (t_stop * t_stop)) / 2) +
                  FRONT_DISTANCE;

  double cost_max = 0.0;
  for (const Obstacle &obstacle : obstacles) {
    // Check if path collide with any obstacle
    // Predict s and d of obstacle in dt
    // TODO margin should come from estimatet obstacle velocity
    if (path.contains(obstacle.s, obstacle.d, 4.0, 1.0)) {
      double distance = obstacle.s - s_stop;
      if (distance < 0) {
        // I want to handle scenario when all paths are going to hit
        // but the one with the largest distance can be selected.
        // For negative distance being closer means higher value here
        // (so at least it will hit withe the minimal impact)
        std::cout << "    id: " << obstacle.id
                  << ", s: " << obstacle.s
                  << ", d: " << obstacle.d
                  << ", dist: " << distance << "\n";
        // nlohmann::json j;
        // j["s"] = path.s;
        // j["d"] = path.d;
        // std::cout << "      s: " << j["s"].dump() << "\n";
        // std::cout << "      d: " << j["d"].dump() << "\n";
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
    // return (10.0 - path.d.back()) / 10.0;
    return 0;
  }
  return 0;
}

//TODO get rid of hardcoded nums (4.0 is the lane width)
double evaluate_keep_lane_center(Path path) {
  double dist = 0;
  double d = path.d.back();
  if (0.0 <= d && d < 4.0) {
    dist = 2.0 - d;
  } else if (4.0 <= d && d < 8.0) {
    dist = 6.0 - d;
  } else if (8.0 <= d && d < 12.0) {
    dist = 10.0 - d;
  }

  // return sigmoid(dist);
  return 0;
}

#endif