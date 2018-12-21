#ifndef COST_H
#define COST_H

#include <math.h>
#include <iostream>
#include "constraints.h"
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
  double s_stop = path.s.front() +
                  (v_max * t_stop) - ((BRAKING_DECC * (t_stop * t_stop)) / 2) +
                  FRONT_DISTANCE;

  double cost_max = 0.0;
  for (const Obstacle &obstacle : obstacles) {
    // Check if path collide with any obstacle
    if (path.contains_d(obstacle.d, 2.0) && path.contains_s(obstacle.s)) {
      double distance = obstacle.s - s_stop;
      if (distance < 0) {
        // I want to handle scenario when all paths are going to hit
        // but the one with the largest distance can be selected
        // For negative distance being closer means higher value
        cost_max = 1 + abs(distance);
      }
    }
  }
  return cost_max;
}

#endif