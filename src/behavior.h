#include "cost.h"
#include "obstacle.h"
#include "path.h"
#include "state.h"

const double SPEED_LIMIT_WEIGHT = 10.0;
const double MAX_ACC_WEIGHT = 0.0;
const double EFFICIENCY_WEIGHT = 1.0;
const double CRASH_WEIGHT = 20.0;
const double OFFROAD_WEIGHT = 1.0;
const double KEEP_LANE_CENTER = 0.1;

class BehaviorPlanner {
 private:
  std::string state = "KL";
  std::vector<Obstacle> obstacles;
  Localization localization;
  Path path_prev;

  std::vector<Path> build_paths(const std::string &state);
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

std::vector<Path> BehaviorPlanner::build_paths(const std::string &state) {
  if (state == "KL") {
    KL state;
    return state.build_paths(this->localization, this->path_prev, this->obstacles);
  }
  if (state == "LCL") {
    LCL state;
    return state.build_paths(this->localization, this->path_prev, this->obstacles);
  }
  // if (state == "LCR") {
  //   LCR state;
  //   return state.build_paths(this->localization, this->path_prev, this->obstacles);
  // }

  return {};
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

  double cost_keep_lane_center = KEEP_LANE_CENTER * evaluate_keep_lane_center(path);
  std::cout << "    klc: " << cost_keep_lane_center << "\n";

  double cost = cost_speed_limit +
                cost_max_acc +
                cost_efficiency +
                cost_crash +
                cost_offroad +
                cost_keep_lane_center;
  std::cout << "    --: " << cost << "\n";

  return cost;
}

Path BehaviorPlanner::next() {
  std::string state_next;
  double cost_min = std::numeric_limits<double>::infinity();
  Path path_min;

  // This is the crucial part to overcome the latency
  this->localization.speed = this->path_prev.get_velocity(this->localization.s) * MS_TO_MPH;

  std::cout << "\n\n";
  std::cout << "from: " << this->state << "\n";
  std::cout << "  v_s: " << this->localization.speed
            << "  v_d: " << path_prev.get_velocity_d(localization.d)
            << " s: " << this->localization.s
            << " d: " << this->localization.d
            << " yaw: " << this->localization.yaw
            << "\n";

  for (const std::string &transition : STATES[this->state]) {
    std::cout << "  transition: " << transition << "\n";
    for (Path &path : build_paths(transition)) {
      std::cout << "   path: " << path.label << "\n";

      Path path_ext;

      if (this->path_prev.size() > 20) {
        int i = this->path_prev.size() - 20;
        while (this->path_prev.s[i] < localization.s && i < this->path_prev.size()) {
          path_ext.s.push_back(this->path_prev.s[i]);
          path_ext.d.push_back(this->path_prev.d[i]);
          i++;
        }

        for (int i = 0; i < path.size() - 20; i++) {
          path_ext.s.push_back(path.s[i]);
          path_ext.d.push_back(path.d[i]);
        }

      } else {
        path_ext = path;
      }

      path_ext.label = path.label;

      double cost = evaluate_path(path_ext);
      if (cost < cost_min) {
        cost_min = cost;
        state_next = transition;
        path_min = path_ext;
      }
    };
  }

  // Check if any transition is better than keeping the current state
  // If all transitions have equal cost, just keep the current state

  std::cout << "to: " << state_next << " " << path_min.label << std::endl;

  this->state = state_next;
  this->path_prev = path_min;
  return path_min;
};