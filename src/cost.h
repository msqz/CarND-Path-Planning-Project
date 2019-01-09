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
#include "limits"

double sigmoid(double x) {
  return 1 / (1 + exp(-x));
}

double linear_eq(double s, std::vector<double> values_s, std::vector<double> values_d) {
  double m = (values_d[1]-values_d[0])/(values_s[1]-values_s[0]);
  double d_new = m * (s-values_s[0]) + values_d[0];
  return d_new;
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
    // Ignore when it's moving backwards
    if (prediction.s < prediction.s_original) {
      continue;  
    }

    // Ignore when it's trajectory is not going to reach me
    if (prediction.s + CAR_LENGTH/2 < localization.s - CAR_LENGTH/2) {
      continue; 
    }

    // Ignore when it's out of path range
    if (prediction.s_original >= path.s.back() + CAR_LENGTH/2) {
      continue;  
    }

    // Ignore cars behind me on the same lane
    Localization loc_pred;
    loc_pred.d = prediction.d_original;
    if (loc_pred.get_lane() == localization.get_lane() && 
        prediction.s_original < localization.s - CAR_LENGTH/2) {
      continue;
    }

    // Check if the path crosses trajectory of obstacle
    // (straight line from it's original to predicted position)
    double distance_min = std::numeric_limits<double>::max();
    std::vector<double> pred_s = {prediction.s_original - CAR_LENGTH/2 - FRONT_DISTANCE, prediction.s + CAR_LENGTH/2 + BACK_DISTANCE};
    std::vector<double> pred_d = {prediction.d_original, prediction.d};
    for (int i = 0; i < path.size(); i++) {
    	// Skip segments which are outside the obstacle trajectory
			if (path.s[i] < pred_s[0] || path.s[i] > pred_s[1]) {
				continue;
			}
      double d_left = linear_eq(path.s[i], pred_s, pred_d) - CAR_WIDTH/2;
      double d_right = linear_eq(path.s[i], pred_s, pred_d) + CAR_WIDTH/2;
      double d_left_my = path.d[i] - CAR_WIDTH/2;
      double d_right_my = path.d[i] + CAR_WIDTH/2;
      if (d_left <= d_left_my && d_left_my <= d_right ||
          d_left <= d_right_my && d_right_my <= d_right){
        //CRASH!!!!!! Distance from current s,d to crashing s,d
        double delta_s = path.s[i] - localization.s;
        double delta_d = path.d[i] - localization.d;
        double distance = std::sqrt(pow(delta_s, 2) + pow(delta_d, 2));

        if (distance < distance_min) {
          distance_min = distance;
        }        
      }
    }

    if (distance_min < std::numeric_limits<double>::max()) {
      std::cout << "{";
      std::cout << "\"id\": " << prediction.id;
      std::cout << ", \"s\": " << prediction.s;
      std::cout << ", \"d\": " << prediction.d;
      std::cout << ", \"dist\": " << distance_min;
      std::cout << "},"; 

      double cost = 999;
      if (distance_min != 0) {
        cost = 1 / distance_min;
      }
      if (cost > cost_max) {
        cost_max = cost;
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

double evaluate_predictability(Path path, Path path_prev) {
  if (path.size() == 0 || path_prev.size() == 0) {
    return 0;
  }
  Localization prev;
  prev.d = path_prev.d.back();
  Localization curr;
  curr.d = path.d.back();
  
  return std::abs(prev.get_lane() - curr.get_lane());
}

#endif
