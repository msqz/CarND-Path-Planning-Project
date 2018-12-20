#ifndef PATH_H
#define PATH_H

#include <vector>
#include "constraints.h"

struct Path {
  std::vector<double> s;
  std::vector<double> d;

  double get_velocity(int at = 0) {
    // it's distance / (50 elements * 0.02s each) -> distance/1s
    // if (at != 0) {
    //   return (this->s[at] - this->s[at-1]) / DELTA_T;
    // }

    if (this->size() < 50) {
      return 0;
    }
    return this->s[49] - this->s[0];
    // return (this->s[this->size()-1] - this->s[this->size()-2]) /DELTA_T;
  }

  int size() {
    return this->s.size();
  }
};

#endif