#include <assert.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "state.h"

using namespace std;

void test_AccState() {
  AccState state = AccState();
  Localization localization = {
      .x = 0,
      .y = 0,
      .s = 0,
      .d = 0,
      .yaw = 0,
      .speed = 0,
  };

  Trajectory t1 = state.build_trajectory(localization);

  assert(abs(t1.s[0] - 0.2) < 0.001);
  assert(abs(t1.s[4] - 5.0) < 0.001);
  assert(abs(t1.s[9] - 20.0) < 0.001);

  localization.s = t1.s[9];
  localization.speed = 20 / MPH_TO_MS;
  Trajectory t2 = state.build_trajectory(localization);

  assert(abs(t2.s[0] - 24.2) < 0.001);
  assert(abs(t2.s[4] - 45.0) < 0.001);
  assert(abs(t2.s[9] - 80.0) < 0.001);
}

void test_CruiseState() {
  CruiseState state = CruiseState();
  Localization localization = {
      .x = 0,
      .y = 0,
      .s = 0,
      .d = 0,
      .yaw = 0,
      .speed = 0,
  };

  Trajectory t1 = state.build_trajectory(localization);
  assert(abs(t1.s[0] - 0) < 0.001);
  assert(abs(t1.s[9] - 0) < 0.001);

  localization.speed = 20 / MPH_TO_MS;
  Trajectory t2 = state.build_trajectory(localization);
  assert(abs(t2.s[0] - 4) < 0.001);
  assert(abs(t2.s[9] - 40) < 0.001);
}

int main() {
  test_AccState();
  test_CruiseState();
  cout << "done\n";

  return 0;
}