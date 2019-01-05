#include <iostream>
#include <assert.h>
#include <chrono>
#include <thread>
#include "tracking.h"

void test_predict() {
  Tracking tracking;
  Obstacle obstacle;
  obstacle.id = 1;

  obstacle.s = 0;
  tracking.add(obstacle);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  obstacle.s = 1;
  tracking.add(obstacle);

  std::this_thread::sleep_for(std::chrono::seconds(2));
  obstacle.s = 2;
  tracking.add(obstacle);

  auto prediction = tracking.predict();
}

int main() {
  test_predict();
  std::cout << "done" << std::endl;
  return 0;
}