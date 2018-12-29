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

  Path t1 = state.build_path(localization);

  assert(abs(t1.s[0] - 0.0018) < 0.0001);
  assert(abs(t1.s[4] - 0.045) < 0.0001);
  assert(abs(t1.s[9] - 0.18) < 0.0001);
  assert(abs(t1.s[49] - 4.5) < 0.0001);

  localization.s = t1.s[49];
  localization.speed = 10 * MS_TO_MPH;
  Path t2 = state.build_path(localization);

  assert(abs(t2.s[0] - 4.7018) < 0.0001);
  assert(abs(t2.s[4] - 5.545) < 0.0001);
  assert(abs(t2.s[9] - 6.68) < 0.0001);
  assert(abs(t2.s[49] - 19.0) < 0.0001);
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

  Path t1 = state.build_path(localization);
  assert(abs(t1.s[0] - 0) < 0.001);
  assert(abs(t1.s[49] - 0) < 0.001);

  localization.speed = 20 * MS_TO_MPH;
  Path t2 = state.build_path(localization);
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

  Path t1 = state.build_path(localization);
  assert(abs(t1.get_velocity() - 10) < 0.001);
}

void test_DeccState() {
  DeccState state;
  Localization localization = {
      .x = 0,
      .y = 0,
      .s = 100,
      .d = 0,
      .yaw = 0,
      .speed = 10 * MS_TO_MPH,
  };

  Path p1 = state.build_path(localization);
  assert(abs(p1.s[0] - 100.1982) < 0.001);
}

void test_RightState() {
  RightState state;
  Localization localization = {
      .x = 0,
      .y = 0,
      .s = 100,
      .d = 0,
      .yaw = 0,
      .speed = 10 * MS_TO_MPH,
  };
}

int main() {
  test_AccState();
  test_CruiseState();
  test_DeccState();
  test_RightState();
  cout << "done\n";

  return 0;
}