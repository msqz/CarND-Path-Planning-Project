#ifndef PATH_H
#define PATH_H

#include <vector>

struct Path {
  std::vector<double> s;
  std::vector<double> d;

  double get_velocity() {
    // it's distance / (50 elements * 0.02s each) -> distance/1s
    if (this->size() < 50) {
      return 0;
    }
    return this->s[49] - this->s[0];
  }

  int size() {
    return this->s.size();
  }
};

#endif