#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <algorithm>
#include "cost.h"
#include "json.hpp"
#include "obstacle.h"
#include "path.h"
#include "state.h"
#include "tracking.h"
#include "trajectory_generator.h"

const double OFFROAD_WEIGHT = 10000.0;
const double CRASH_WEIGHT = 1000.0;
const double SPEED_LIMIT_WEIGHT = 100.0;
const double MAX_ACC_WEIGHT = 10.0;
const double EFFICIENCY_WEIGHT = 2.0;

const double KEEP_RIGHT_WEIGHT = 0.0;
const double PREDICTABILITY = 0.01;

class BehaviorPlanner {
 private:
  std::string state_s = "STOP";
  std::string state_d = "STRAIGHT";
  std::vector<Obstacle> obstacles;
  Localization localization;
  Path path_prev;
  Tracking tracking;

  Path build_path(const std::string &state);
  double evaluate_path(const Path &path);

 public:
  void set_obstacles(const std::vector<Obstacle> &obstacles);
  void set_localization(const Localization &localization);

  Path next(TrajectoryGenerator generator, Trajectory trajectory_prev, double end_path_s);
};

void BehaviorPlanner::set_obstacles(const std::vector<Obstacle> &obstacles) {
  this->obstacles = obstacles;
  for (const Obstacle &o : obstacles) {
    tracking.add(o);
  }
}

void BehaviorPlanner::set_localization(const Localization &localization) {
  this->localization = localization;
}

Path BehaviorPlanner::build_path(const std::string &state) {
  Path path;

  if (state == "CRUISE") {
    CruiseState state = CruiseState();
    return state.build_path(this->localization, this->path_prev);
  }
  if (state == "STOP") {
    StopState state = StopState();
    return state.build_path(this->localization, this->path_prev);
  }
  if (state == "STRAIGHT") {
    StraightState state = StraightState();
    return state.build_path(this->localization, this->path_prev);
  }
  if (state == "LEFT") {
    LeftState state = LeftState();
    return state.build_path(this->localization, this->path_prev);
  }
  if (state == "RIGHT") {
    RightState state = RightState();
    return state.build_path(this->localization, this->path_prev);
  }
  if (state == "DOUBLE_LEFT") {
    DoubleLeftState state = DoubleLeftState();
    return state.build_path(this->localization, this->path_prev);
  }
  if (state == "DOUBLE_RIGHT") {
    DoubleRightState state = DoubleRightState();
    return state.build_path(this->localization, this->path_prev);
  }
  if (state == "ACC_SOFT") {
    AccState state = AccState();
    return state.build_path(this->localization, this->path_prev, SOFT_ACC_RATE);
  }
  if (state == "ACC_MED") {
    AccState state = AccState();
    return state.build_path(this->localization, this->path_prev, MED_ACC_RATE);
  }
  if (state == "ACC_HARD") {
    AccState state = AccState();
    return state.build_path(this->localization, this->path_prev, HARD_ACC_RATE);
  }
  if (state == "DECC_SOFT") {
    DeccState state = DeccState();
    return state.build_path(this->localization, this->path_prev, SOFT_DECC_RATE);
  }
  if (state == "DECC_MED") {
    DeccState state = DeccState();
    return state.build_path(this->localization, this->path_prev, MED_DECC_RATE);
  }
  if (state == "DECC_HARD") {
    DeccState state = DeccState();
    return state.build_path(this->localization, this->path_prev, HARD_DECC_RATE);
  }

  return path;
}

double BehaviorPlanner::evaluate_path(const Path &path) {
  std::cout << ", \"evaluation\": ";
  std::cout << "{";

  double cost_speed_limit = SPEED_LIMIT_WEIGHT * evaluate_speed_limit(path);
  double cost_max_acc = MAX_ACC_WEIGHT * evaluate_max_acc(path, this->localization);
  double cost_efficiency = EFFICIENCY_WEIGHT * evaluate_efficiency(path);

  std::cout << "\"crash\": ";
  std::cout << "[";
  double cost_crash = CRASH_WEIGHT * evaluate_crash(path, this->localization, this->tracking.predict());
  std::cout << "]";

  double cost_offroad = OFFROAD_WEIGHT * evaluate_offroad(path);
  double cost_keep_right = KEEP_RIGHT_WEIGHT * evaluate_keep_right(path);
  double cost_predictability = PREDICTABILITY * evaluate_predictability(path, this->path_prev);

  double cost = cost_speed_limit +
                cost_max_acc +
                cost_efficiency +
                cost_crash +
                cost_offroad +
                cost_keep_right +
                cost_predictability;

  std::cout << ", \"cost\": ";
  std::cout << "{";
  std::cout << "\"sl\": " << cost_speed_limit;
  std::cout << ", \"ma\": " << cost_max_acc;
  std::cout << ", \"ef\": " << cost_efficiency;
  std::cout << ", \"cr\": " << cost_crash;
  std::cout << ", \"of\": " << cost_offroad;
  std::cout << ", \"kr\": " << cost_keep_right;
  std::cout << ", \"pr\": " << cost_predictability;
  std::cout << ", \"total\": " << cost;
  std::cout << "}";
  std::cout << "}";

  return cost;
}

Path BehaviorPlanner::next(TrajectoryGenerator generator, Trajectory trajectory_prev, double end_path_s) {
  std::string state_s_next;
  std::string state_d_next;
  double cost_min = std::numeric_limits<double>::max();
  Path path_min;

  // This is the crucial part to overcome the latency
  this->localization.speed = this->path_prev.get_velocity(this->localization.s) * MS_TO_MPH;
  for (int i = 1; i < this->path_prev.size(); i++) {
    if (this->path_prev.s[i - 1] <= this->localization.s &&
        this->localization.s <= this->path_prev.s[i]) {
      this->localization.d = this->path_prev.d[i];
      break;
    }
  }

  std::cout << ", \"from\": \"" << this->state_s << "/" << this->state_d << "\"";
  std::cout << ", \"v_s\": " << this->localization.speed;
  std::cout << ", \"a_s\": " << path_prev.get_acc_s(localization.s);
  std::cout << ", \"v_d\": " << path_prev.get_velocity_d(localization.d);
  std::cout << ", \"a_d\": " << path_prev.get_acc_d(localization.d);
  std::cout << ", \"s\": " << this->localization.s;
  std::cout << ", \"d\": " << this->localization.d;
  std::cout << ", \"x\": " << this->localization.x;
  std::cout << ", \"y\": " << this->localization.y;

  std::cout << ", \"predictions\": ";
  std::cout << "[";
  for (Prediction p : this->tracking.predict()) {
    std::cout << "{";
    std::cout << "\"id\": " << p.id;
    std::cout << ", \"s\": " << p.s;
    std::cout << ", \"d\": " << p.d;
    std::cout << ", \"s_orig\": " << p.s_original;
    std::cout << ", \"d_orig\": " << p.d_original;
    std::cout << "},";
  }
  std::cout << "]";

  std::map<std::tuple<std::string, std::string>, std::tuple<double, Path>> state_to_cost;
  double cost_avg = 0;

  std::cout << ", \"transitions\": ";
  std::cout << "[";

  for (const std::string &transition_s : STATES_S[this->state_s]) {
    for (const std::string &transition_d : STATES_D[this->state_d]) {
      Path path_s = build_path(transition_s);
      Path path_d = build_path(transition_d);
      Path path;
      path.s = path_s.s;
      path.d = path_d.d;

      path.trajectory = generator.generate(path, trajectory_prev, localization);

      std::cout << "{";
      std::cout << " \"transition\": \"" << transition_s << "/" << transition_d << "\"";
      std::cout << ", \"s\": " << path.s.back();
      std::cout << ", \"d\": " << path.d.back();
      double cost = this->evaluate_path(path);
      state_to_cost[{transition_s, transition_d}] = {cost, path};
      cost_avg += cost;

      json j;
      j["next_s"] = path.s;
      j["next_d"] = path.d;
      std::cout << ", \"path_s\": " << j["next_s"].dump();
      std::cout << ", \"path_d\": " << j["next_d"].dump();

      std::cout << "},";
    }
  }
  cost_avg /= STATES_S[this->state_s].size() * STATES_D[this->state_d].size();

  // Check if any transition is better that keeping the current state
  bool equal = true;
  for (const std::string &transition_s : STATES_S[this->state_s]) {
    for (const std::string &transition_d : STATES_D[this->state_d]) {
      double cost = std::get<0>(state_to_cost[{transition_s, transition_d}]);
      if (cost != cost_avg) {
        equal = false;
        if (cost < cost_min) {
          cost_min = cost;
          state_s_next = transition_s;
          state_d_next = transition_d;
          path_min = std::get<1>(state_to_cost[{state_s_next, state_d_next}]);
        }
      }
    }
  }

  // If all transitions have equal cost, just keep the current state
  if (equal == true) {
    state_s_next = this->state_s;
    state_d_next = this->state_d;
    path_min = std::get<1>(state_to_cost[{state_s_next, state_d_next}]);
  }

  std::cout << "]";
  std::cout << ", \"to\": \"" << state_s_next << "/" << state_d_next << "\"";
  this->state_s = state_s_next;
  this->state_d = state_d_next;
  this->path_prev = path_min;
  return path_min;
};

#endif
