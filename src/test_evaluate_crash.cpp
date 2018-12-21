#include <assert.h>
#include <iostream>
#include <vector>
#include "constraints.h"
#include "cost.h"
#include "localization.h"
#include "obstacle.h"
#include "path.h"

// v=30m/s requires 64.125m + 4.0m of FRONT_DISTANCE to stop completely
Path build_path(double s, double v) {
  Path path;
  for (int i = 0; i < PATH_LENGTH; i++) {
    path.s.push_back(s + i * v * DELTA_T);
    path.d.push_back(0);
  }
  return path;
}

void given_insufficient_distance_should_return_1() {
  Localization localization;
  localization.s = 0;
  double v = 30;
  Path path = build_path(localization.s, v);
  Obstacle obstacle;
  obstacle.s = 64.125 + FRONT_DISTANCE - 1;
  std::vector<Obstacle> obstacles{obstacle};

  double cost = evaluate_crash(path, localization, obstacles);

  assert(abs(cost - 1.0) < 0.001);
}

void given_sufficient_distance_should_return_0() {
  Localization localization;
  localization.s = 0;
  double v = 30;
  Path path = build_path(localization.s, v);
  Obstacle obstacle;
  obstacle.s = 64.125 + FRONT_DISTANCE + 1;
  std::vector<Obstacle> obstacles{obstacle};

  double cost = evaluate_crash(path, localization, obstacles);

  assert(abs(cost - 0.0) < 0.001);
}

int main() {
  given_insufficient_distance_should_return_1();
  given_sufficient_distance_should_return_0();
  std::cout << "done" << std::endl;

  return 0;
}