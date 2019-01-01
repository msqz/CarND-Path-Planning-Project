#include <assert.h>
#include <iostream>
#include <vector>
#include "json.hpp"
#include "state.h"

void test_JMT_s() {
  std::vector<double> start{
      0.0,
      0.0,
      0.0,
  };
  std::vector<double> end{
      10.0,
      10.0,
      10.0,
  };
  std::vector<double> coeffs = JMT(start, end, 2.0);

  Path path;
  for (int i = 0; i < PATH_LENGTH; i++) {
    double dt = (i + 1) * DELTA_T;
    double s = coeffs[0] +
               coeffs[1] * dt +
               coeffs[2] * pow(dt, 2) +
               coeffs[3] * pow(dt, 3) +
               coeffs[4] * pow(dt, 4) +
               coeffs[5] * pow(dt, 5);
    path.s.push_back(s);
  }

  nlohmann::json j;
  j["path"] = path.s;
  std::cout << j["path"].dump() << std::endl;
  // assert(abs(10.0 - path.s.back()) < 0.0001);
}

void test_JMT_d() {
  std::vector<double> start{
      -4.2,
      0.8,
      0.0,
  };
  std::vector<double> end{
      -2.0,
      0.0,
      0.0,
  };
  std::vector<double> coeffs = JMT(start, end, DELTA_T * PATH_LENGTH);

  Path path;
  for (int i = 0; i < PATH_LENGTH; i++) {
    double dt = (i + 1) * DELTA_T;
    double s = coeffs[0] +
               coeffs[1] * dt +
               coeffs[2] * pow(dt, 2) +
               coeffs[3] * pow(dt, 3) +
               coeffs[4] * pow(dt, 4) +
               coeffs[5] * pow(dt, 5);
    path.s.push_back(s);
  }

  nlohmann::json j;
  j["path"] = path.s;
  std::cout << j["path"].dump() << std::endl;
  // assert(abs(10.0 - path.s.back()) < 0.0001);
};

int main() {
  test_JMT_s();
  test_JMT_d();
  std::cout << "done" << std::endl;
  return 0;
}