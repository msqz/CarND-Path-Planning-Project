
#include "car.h"
#include <math.h>
#include <string.h>
#include <functional>
#include <limits>
#include <map>
#include <vector>
#include "constraints.h"
#include "state.h"

Car::Car(std::map<std::string, std::vector<std::string>> states, std::string state) {
  this->states = states;
  this->state = state;
}

void Car::set_localization(Localization localization) {
  this->localization = localization;
}

std::string Car::get_state() {
  return this->state;
}

double evaluate_speed_limit(Trajectory trajectory) {
  double v_target = SPEED_LIMIT - BUFFER_V;
  double v_trajectory = trajectory.get_velocity() * MS_TO_MPH;
  std::cout << "      " << v_trajectory << " / " << v_target << "\n";

  if (v_trajectory > v_target) {
    return (v_trajectory - v_target) / BUFFER_V;
  }

  return (v_target - v_trajectory) / v_target;
}

double evaluate_max_acc(Trajectory trajectory) {
  if (trajectory.s.size() < 2) {
    return 0;
  }
  double s = trajectory.s[1] - trajectory.s[0];
  double a = (2 * s) / pow(DELTA_T, 2);

  // if (a > MAX_ACC) {
  //   return 1;
  // }
  return exp(a - MAX_ACC);
}

double evaluate_efficiency(Trajectory trajectory) {
  return std::min(1.0, exp(-trajectory.get_velocity()));
}

double evaluate(Trajectory trajectory) {
  double cost_speed_limit = evaluate_speed_limit(trajectory);
  std::cout << "    sl: " << cost_speed_limit << "\n";
  // double cost_max_acc = evaluate_max_acc(trajectory);
  // std::cout << "    ma: " << cost_max_acc << "\n";
  double cost_efficiency = evaluate_efficiency(trajectory);
  std::cout << "    ef: " << cost_efficiency << "\n";
  // other costs
  double cost = cost_speed_limit;
  std::cout << "    --: " << cost << "\n";
  return cost;
}

Trajectory Car::build_trajectory(std::string state) {
  Trajectory trajectory;

  if (state == "ACC") {
    AccState state = AccState();
    return state.build_trajectory(this->localization);
  } else if (state == "CRUISE") {
    CruiseState state = CruiseState();
    return state.build_trajectory(this->localization);
  } else if (state == "DECC") {
    DeccState state = DeccState();
    return state.build_trajectory(this->localization);
  } else {
    for (int i = 0; i < 50; i++) {
      trajectory.s.push_back(this->localization.s);
      trajectory.d.push_back(this->localization.d);
    }
  }

  return trajectory;
}

Trajectory Car::get_trajectory() {
  std::string state_next;
  double cost_min = std::numeric_limits<double>::infinity();
  // TODO Handle equal costs - cruise then.
  Trajectory trajectory_min;
  std::cout << "from: " << this->state << "\n";
  for (const std::string &transition : this->states[this->state]) {
    Trajectory trajectory = build_trajectory(transition);
    std::cout << "  transition: " << transition << ":\n";
    bool equal = false;
    double cost = evaluate(trajectory);
    if (cost < cost_min) {
      cost_min = cost;
      trajectory_min = trajectory;
      state_next = transition;
    }
  }
  std::cout << "to:   " << state_next << "\n";
  this->state = state_next;
  return trajectory_min;
}

double Trajectory::get_velocity() {
  double distance = this->s.back() - this->s[0];
  double t = this->s.size() * DELTA_T;
  return distance / t;
}