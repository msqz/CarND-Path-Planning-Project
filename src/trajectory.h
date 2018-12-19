#include <vector>

struct Trajectory {
  std::vector<double> x;
  std::vector<double> y;

  int size() {
    return x.size();
  }
};
