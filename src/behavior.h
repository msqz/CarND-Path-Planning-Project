#include "path.h"
#include "prediction.h"
#include "state.h"

const double SPEED_LIMIT_WEIGHT = 10.0;
const double MAX_ACC_WEIGHT = 2.0;
const double EFFICIENCY_WEIGHT = 1.0;

double sigmoid(double x) {
  return 1 / (1 + exp(-x));
}

class BehaviorPlanner {
 private:
  std::string state = "STOP";
  Prediction prediction;
  Localization localization;
  Localization localization_internal;
  Path path_prev;

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

  if (v_trajectory > SPEED_LIMIT) {
    return 1;
  }
  return 0;
}

double evaluate_max_acc(Path trajectory, Localization localization) {
  for (int i = 2; i < trajectory.size(); i++) {
    double v_0 = (trajectory.s[i - 1] - trajectory.s[i - 2]) / DELTA_T;
    double v_1 = (trajectory.s[i] - trajectory.s[i - 1]) / DELTA_T;
    double a = (v_1 - v_0) / DELTA_T;
    if (a > MAX_ACC) {
      return 1;
    }
  };

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

double evaluate(Path trajectory, Localization localization) {
  double cost_speed_limit = SPEED_LIMIT_WEIGHT * evaluate_speed_limit(trajectory);
  std::cout << "    sl: " << cost_speed_limit << "\n";
  double cost_max_acc = MAX_ACC_WEIGHT * evaluate_max_acc(trajectory, localization);
  std::cout << "    ma: " << cost_max_acc << "\n";
  double cost_efficiency = EFFICIENCY_WEIGHT * evaluate_efficiency(trajectory);
  std::cout << "    ef: " << cost_efficiency << "\n";
  // other costs
  double cost = cost_speed_limit + cost_max_acc + cost_efficiency;
  std::cout << "    --: " << cost << "\n";
  return cost;
}

Path BehaviorPlanner::next() {
  std::string state_next;
  double cost_min = std::numeric_limits<double>::infinity();
  Path path_min;

  //it sleeps 1sec between the cycles
  double point_idx = 0;
  for (int i = 1; i < this->path_prev.size(); i++) {
    if (this->path_prev.s[i-1] <= localization.s && localization.s <= this->path_prev.s[i]) {
      point_idx = i;
    }
  }
  double v_point = 0.0;
  if (point_idx > 0){
    v_point = (this->path_prev.s[point_idx] - this->path_prev.s[point_idx-1]) / DELTA_T;
    this->localization.speed = v_point * MS_TO_MPH;
  }

  std::cout << "\n";
  std::cout << "from: " << this->state << "\n";
  std::cout << "  v: " << this->localization.speed
            << " s: " << this->localization.s
            << " point: " << point_idx
            << " v_point: " << v_point * MS_TO_MPH
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
  std::cout << "to: " << state_next << std::endl;
  this->state = state_next;
  this->localization_internal.speed = path_min.get_velocity(53);
  this->path_prev = path_min;
  return path_min;
};