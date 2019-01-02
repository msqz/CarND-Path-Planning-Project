#include "cost.h"
#include "obstacle.h"
#include "path.h"
#include "state.h"

const double SPEED_LIMIT_WEIGHT = 10.0;
const double MAX_ACC_WEIGHT = 2.0;
const double EFFICIENCY_WEIGHT = 1.0;
const double CRASH_WEIGHT = 20.0;
const double OFFROAD_WEIGHT = 1.0;
const double KEEP_RIGHT_WEIGHT = 0.01;
const double KEEP_LANE_CENTER = 10.0;

class BehaviorPlanner {
 private:
  std::string state_s = "STOP";
  std::string state_d = "STRAIGHT";
  std::vector<Obstacle> obstacles;
  Localization localization;
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
    return state.build_path(this->localization, this->path_prev);
  }
  if (state == "CRUISE") {
    CruiseState state = CruiseState();
    return state.build_path(this->localization, this->path_prev);
  }
  if (state == "DECC") {
    DeccState state = DeccState();
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

  double cost_offroad = OFFROAD_WEIGHT * evaluate_offroad(path);
  std::cout << "    of: " << cost_offroad << "\n";

  double cost_keep_right = KEEP_RIGHT_WEIGHT * evaluate_keep_right(path);
  std::cout << "    kr: " << cost_keep_right << "\n";

  double cost_keep_lane_center = KEEP_LANE_CENTER * evaluate_keep_lane_center(path);
  std::cout << "    klc: " << cost_keep_lane_center << "\n";

  double cost = cost_speed_limit +
                cost_max_acc +
                cost_efficiency +
                cost_crash +
                cost_offroad +
                cost_keep_right +
                cost_keep_lane_center;
  std::cout << "    --: " << cost << "\n";

  return cost;
}

Path BehaviorPlanner::next() {
  std::string state_s_next;
  std::string state_d_next;
  double cost_min = std::numeric_limits<double>::infinity();
  Path path_min;

  // This is the crucial part to overcome the latency
  this->localization.speed = this->path_prev.get_velocity(this->localization.s) * MS_TO_MPH;
  for (int i = 1; i < this->path_prev.size(); i++) {
    if (this->path_prev.s[i - 1] <= this->localization.s && this->localization.s <= this->path_prev.s[i]) {
      this->localization.d = this->path_prev.d[i];
      break;
    }
  }

  std::cout << "\n";
  std::cout << "from: " << this->state_s << "/" << this->state_d << "\n";
  std::cout << "  v_s: " << this->localization.speed
            << " a_s: " << path_prev.get_acc_s(localization.s)
            << " v_d: " << path_prev.get_velocity_d(localization.d)
            << " a_d: " << path_prev.get_acc_d(localization.d)
            << " s: " << this->localization.s
            << " d: " << this->localization.d
            << " yaw: " << this->localization.yaw
            << "\n";

  std::map<std::tuple<std::string, std::string>, std::tuple<double, Path>> state_to_cost;
  double cost_avg = 0;
  for (const std::string &transition_s : STATES_S[this->state_s]) {
    for (const std::string &transition_d : STATES_D[this->state_d]) {
      Path path_s = build_path(transition_s);
      Path path_d = build_path(transition_d);
      Path path;
      path.s = path_s.s;
      path.d = path_d.d;
      std::cout << "  transition: " << transition_s << "/" << transition_d << ":\n ";
      double cost = this->evaluate_path(path);
      state_to_cost[{transition_s, transition_d}] = {cost, path};
      cost_avg += cost;
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
    std::cout << "keep\n";
  }

  std::cout << "to: " << state_s_next << "/" << state_d_next << std::endl;
  this->state_s = state_s_next;
  this->state_d = state_d_next;
  this->path_prev = path_min;
  return path_min;
};