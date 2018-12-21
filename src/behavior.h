#include "path.h"
#include "obstacle.h"
#include "state.h"
#include "cost.h"

const double SPEED_LIMIT_WEIGHT = 10.0;
const double MAX_ACC_WEIGHT = 2.0;
const double EFFICIENCY_WEIGHT = 1.0;
const double CRASH_WEIGHT = 20.0;

class BehaviorPlanner {
 private:
  std::string state = "STOP";
  std::vector<Obstacle> obstacles;
  Localization localization;
  Localization localization_internal;
  Path path_prev;

  Path build_path(const std::string &state);
  double evaluate_path(const Path &path);

 public:
  void set_obstacles(const std::vector<Obstacle> &obstacles);
  void set_localization(const Localization &localization);

  Path next();
};

void BehaviorPlanner::set_obstacles(const std::vector<Obstacle> &obstacles) {
  this->obstacles = obstacles;
}

void BehaviorPlanner::set_localization(const Localization &localization) {
  this->localization = localization;
}

Path BehaviorPlanner::build_path(const std::string &state) {
  Path path;

  if (state == "ACC") {
    AccState state = AccState();
    return state.build_path(this->localization);
  }
  if (state == "CRUISE") {
    CruiseState state = CruiseState();
    return state.build_path(this->localization);
  }
  if (state == "DECC") {
    DeccState state = DeccState();
    return state.build_path(this->localization);
  }
  if (state == "STOP") {
    StopState state = StopState();
    return state.build_path(this->localization);
  }

  return path;
}

double BehaviorPlanner::evaluate_path(const Path &path) {
  double cost_speed_limit = SPEED_LIMIT_WEIGHT * evaluate_speed_limit(path);
  std::cout << "    sl: " << cost_speed_limit << "\n";
  
  double cost_max_acc = MAX_ACC_WEIGHT * evaluate_max_acc(path, this->localization);
  std::cout << "    ma: " << cost_max_acc << "\n";
  
  double cost_efficiency = EFFICIENCY_WEIGHT * evaluate_efficiency(path);
  std::cout << "    ef: " << cost_efficiency << "\n";
  
  double cost_crash = CRASH_WEIGHT * evaluate_crash(path, this->localization, this->obstacles);
  std::cout << "    cr: " << cost_crash << "\n";
  
  double cost = cost_speed_limit + cost_max_acc + cost_efficiency + cost_crash;
  std::cout << "    --: " << cost << "\n";
  return cost;
}

Path BehaviorPlanner::next() {
  std::string state_next;
  double cost_min = std::numeric_limits<double>::infinity();
  Path path_min;

  // This is the crucial part to overcome the latency
  this->localization.speed = this->path_prev.get_velocity(this->localization.s) * MS_TO_MPH;

  std::cout << "\n";
  std::cout << "from: " << this->state << "\n";
  std::cout << "  v: " << this->localization.speed
            << " s: " << this->localization.s
            << "\n";

  if (STATES[this->state].size() == 1) {
    state_next = STATES[this->state][0];
    path_min = this->build_path(state_next);
  } else {
    std::map<std::string, std::tuple<double, Path>> state_to_cost;
    double cost_avg = 0;
    for (const std::string &transition : STATES[this->state]) {
      Path path = build_path(transition);
      std::cout << "  transition: " << transition << ":\n";
      double cost = this->evaluate_path(path);
      state_to_cost[transition] = {cost, path};
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
          path_min = std::get<1>(state_to_cost[state_next]);
        }
      }
    }

    // If all transitions have equal cost, just keep the current state
    if (equal == true) {
      state_next = this->state;
      path_min = std::get<1>(state_to_cost[state_next]);
      std::cout << "keep\n";
    }
  }
  std::cout << "to: " << state_next << std::endl;
  this->state = state_next;
  this->localization_internal.speed = path_min.get_velocity(53);
  this->path_prev = path_min;
  return path_min;
};