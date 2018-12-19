#include "path.h"
#include "prediction.h"
#include "state.h"

class BehaviorPlanner {
 private:
  std::string state = "STOP";
  Prediction prediction;
  Localization localization;

  Path build_path(const std::string &state);

 public:
  void set_prediction(const Prediction prediction);
  void set_localization(const Localization localization);

  Path next();
};

void BehaviorPlanner::set_prediction(const Prediction prediction) {
  this->prediction = prediction;
}

void BehaviorPlanner::set_localization(const Localization localization) {
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

double evaluate_speed_limit(Path trajectory) {
  double v_target = SPEED_LIMIT - BUFFER_V;
  double v_trajectory = trajectory.get_velocity() * MS_TO_MPH;
  std::cout << "    v: " << v_trajectory << "\n";
  
  if (v_trajectory > v_target) {
    return (v_trajectory - v_target) / BUFFER_V;
  }

  return STOP_COST * (v_target - v_trajectory) / v_target;
}

double evaluate_max_acc(Path trajectory, Localization localization) {
  double v_0 = localization.speed * MPH_TO_MS;
  double v_1 = (trajectory.s[0] - localization.s) / DELTA_T;
  double a = (v_1 - v_0) / DELTA_T;

  return exp(a - MAX_ACC);
}

double evaluate_efficiency(Path trajectory) {
  // time to reach the goal - map waypoint?
  return std::min(1.0, exp(-trajectory.get_velocity()));
}


double evaluate(Path trajectory, Localization localization) {
  double cost_speed_limit = evaluate_speed_limit(trajectory);
  std::cout << "    sl: " << cost_speed_limit << "\n";
  double cost_max_acc = evaluate_max_acc(trajectory, localization);
  std::cout << "    ma: " << cost_max_acc << "\n";
  double cost_efficiency = evaluate_efficiency(trajectory);
  std::cout << "    ef: " << cost_efficiency << "\n";
  // other costs
  double cost = cost_speed_limit + cost_max_acc + cost_efficiency;
  std::cout << "    --: " << cost << "\n";
  return cost;
}

// Try building on top of prev_path_s and _d
Path BehaviorPlanner::next() {
  std::string state_next;
  double cost_min = std::numeric_limits<double>::infinity();
  Path path_min;

  std::cout << "\n";
  std::cout << "from: " << this->state << "\n";
  std::cout << "  v: " << this->localization.speed << " yaw: " << this->localization.yaw << "\n";

  if (STATES[this->state].size() == 1) {
    state_next = STATES[this->state][0];
    path_min = this->build_path(state_next);
  } else {
    std::map<std::string, std::tuple<double, Path>> state_to_cost;
    double cost_avg = 0;
    for (const std::string &transition : STATES[this->state]) {
      Path path = build_path(transition);
      std::cout << "  transition: " << transition << ":\n";
      double cost = evaluate(path, this->localization);
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
  std::cout << "to: " << state_next << "\n";
  this->state = state_next;
  std::cout<<"state set to: "<<this->state<<"\n";

  return path_min;
};