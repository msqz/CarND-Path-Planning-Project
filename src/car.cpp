
#include "car.h"
#include <math.h>
#include <string.h>
#include <functional>
#include <limits>
#include <map>
#include <vector>
#include "constraints.h"
#include "state.h"

Car::Car() {
  this->state = "STOP";
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
  std::cout << "    v: " << v_trajectory << "\n";
  
  if (v_trajectory > v_target) {
    return (v_trajectory - v_target) / BUFFER_V;
  }

  return STOP_COST * (v_target - v_trajectory) / v_target;
}

double evaluate_max_acc(Trajectory trajectory) {
  if (trajectory.s.size() < 2) {
    return 0;
  }
  double s = trajectory.s[1] - trajectory.s[0];
  double a = (2 * s) / pow(DELTA_T, 2);

  return exp(a - MAX_ACC);
}

double evaluate_efficiency(Trajectory trajectory) {
  // time to reach the goal - map waypoint?
  return std::min(1.0, exp(-trajectory.get_velocity()));
}

double evaluate(Trajectory trajectory) {
  double cost_speed_limit = evaluate_speed_limit(trajectory);
  std::cout << "    sl: " << cost_speed_limit << "\n";
  // double cost_max_acc = evaluate_max_acc(trajectory);
  // std::cout << "    ma: " << cost_max_acc << "\n";
  // double cost_efficiency = evaluate_efficiency(trajectory);
  // std::cout << "    ef: " << cost_efficiency << "\n";
  // other costs
  double cost = cost_speed_limit;  // + cost_efficiency;
  std::cout << "    --: " << cost << "\n";
  return cost;
}

Trajectory Car::build_trajectory(std::string state) {
  Trajectory trajectory;

  if (state == "ACC") {
    AccState state = AccState();
    return state.build_trajectory(this->localization);
  }
  if (state == "CRUISE") {
    CruiseState state = CruiseState();
    return state.build_trajectory(this->localization);
  }
  if (state == "DECC") {
    DeccState state = DeccState();
    return state.build_trajectory(this->localization);
  }
  if (state == "STOP") {
    StopState state = StopState();
    return state.build_trajectory(this->localization);
  }

  return trajectory;
}

//TODO remove setting state here
Trajectory Car::get_trajectory() {
  std::string state_next;
  double cost_min = std::numeric_limits<double>::infinity();
  Trajectory trajectory_min;

  std::cout << "\n";
  std::cout << "from: " << this->state << "\n";
  std::cout << "  v: " << this->localization.speed << " yaw: " << this->localization.yaw << "\n";

  if (STATES[this->state].size() == 1) {
    state_next = STATES[this->state][0];
    Trajectory trajectory_min = build_trajectory(state_next);
  } else {
    std::map<std::string, std::tuple<double, Trajectory>> state_to_cost;
    double cost_avg = 0;
    for (const std::string &transition : STATES[this->state]) {
      Trajectory trajectory = build_trajectory(transition);
      std::cout << "  transition: " << transition << ":\n";
      double cost = evaluate(trajectory);
      state_to_cost[transition] = {cost, trajectory};
      cost_avg += cost;
    }
    cost_avg /= STATES[this->state].size();

    // Check if any transition is better that keeping the current state
    bool equal = true;
    for (const std::string &transition : STATES[this->state]) {
      double cost = std::get<0>(state_to_cost[transition]);
      if (cost != cost_avg) {
        equal = false;
        if (cost < cost_min) {
          cost_min = cost;
          state_next = transition;
          trajectory_min = std::get<1>(state_to_cost[state_next]);
        }
      }
    }

    // If all transitions have equal cost, just keep the current state
    if (equal == true) {
      state_next = this->state;
      trajectory_min = std::get<1>(state_to_cost[state_next]);
      std::cout << "keep\n";
    }
  }
  std::cout << "to: " << state_next << "\n";
  this->state = state_next;
  return trajectory_min;
}

double Trajectory::get_velocity() {
  // it's distance / (50 elements * 0.02s each) -> distance/1s
  return this->s[49] - this->s[0];
}