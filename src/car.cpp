#include "car.h"
#include <math.h>
#include <string.h>
#include <functional>
#include <limits>
#include <map>
#include <vector>

const double MAX_ACC = 10;
const double DELTA_T = 0.2;
const double SPEED_LIMIT = 50;
const double BUFFER_V = 5;

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
  double v_trajectory = trajectory.get_velocity();
  
  if (v_trajectory > SPEED_LIMIT) {
    return 1.0;
  }
  if (v_trajectory > v_target && v_trajectory <= SPEED_LIMIT) {
    return (v_trajectory - v_target) / BUFFER_V;
  }
  
  return (v_target - v_trajectory) / v_target; 
}

double evaluate_max_acc(Trajectory trajectory) {
  return 0;
}

double evaluate_efficiency(Trajectory trajectory) {
  return std::min(1.0, exp(-trajectory.get_velocity()));
}

double evaluate(Trajectory trajectory) {
  double cost = 0;
  cost += 100 * evaluate_speed_limit(trajectory);
  cost += evaluate_max_acc(trajectory);
  cost += evaluate_efficiency(trajectory);
  // other costs
  return cost;
}

Trajectory Car::build_trajectory(std::string state) {
  Trajectory trajectory;

  if (state == "ACC") {
    double a = MAX_ACC * DELTA_T;
    double s = this->localization.s;
    for (int i = 0; i < 10; i++) {
      double t = i * DELTA_T;
      double delta_s = (a * std::pow(t, 2)) / 2;
      s += delta_s;
      trajectory.s.push_back(s);
      trajectory.d.push_back(this->localization.d);
    }
  } else if (state == "CRUISE") {
    double s = this->localization.s;
    for (int i = 0; i < 50; i++) {
      double delta_s = this->localization.speed * DELTA_T;
      s += delta_s;
      trajectory.s.push_back(s);
      trajectory.d.push_back(this->localization.d);
    }
  } else {
    trajectory.s.push_back(this->localization.s);
    trajectory.d.push_back(this->localization.d);
  }

  return trajectory;
}

Trajectory Car::get_trajectory() {
  std::string state_next;
  double cost_min = std::numeric_limits<double>::infinity();
  Trajectory trajectory_min;
  for (const std::string &transition : this->states[this->state]) {
    Trajectory trajectory = build_trajectory(transition);
    double cost = evaluate(trajectory);
    if (cost < cost_min) {
      cost_min = cost;
      trajectory_min = trajectory;
      state_next = transition;
    }
  }

  this->state = state_next;
  return trajectory_min;
}

double Trajectory::get_velocity() {
  double distance = this->s.back() - this->s[0];
  double t = this->s.size();
  return distance / t;
}