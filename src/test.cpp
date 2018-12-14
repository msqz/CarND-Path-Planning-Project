#include <assert.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "constraints.h"
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

  assert(abs(t1.s[0] - 0.002) < 0.0001);
  assert(abs(t1.s[4] - 0.05) < 0.0001);
  assert(abs(t1.s[9] - 0.2) < 0.0001);
  assert(abs(t1.s[49] - 5.0) < 0.0001);

  localization.s = t1.s[49];
  localization.speed = 10 * MS_TO_MPH;
  Trajectory t2 = state.build_trajectory(localization);

  assert(abs(t2.s[0] - 5.202) < 0.0001);
  assert(abs(t2.s[4] - 6.05) < 0.0001);
  assert(abs(t2.s[9] - 7.2) < 0.0001);
  assert(abs(t2.s[49] - 20.0) < 0.0001);
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
  assert(abs(t1.s[49] - 0) < 0.001);

  localization.speed = 20 * MS_TO_MPH;
  Trajectory t2 = state.build_trajectory(localization);
  assert(abs(t2.s[0] - 0.4) < 0.001);
  assert(abs(t2.s[49] - 20) < 0.001);
}

void test_Trajectory_get_velocity() {
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
  assert(abs(t1.get_velocity() - 10) < 0.001);
}

int main() {
  test_AccState();
  test_CruiseState();
  cout << "done\n";

  return 0;
}