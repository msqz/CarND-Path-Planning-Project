#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>

struct Obstacle {
  int id;
  double x;
  double y;
  double v_x;
  double v_y;
  double s;
  double d;

  //TODO unify with location
  int get_lane() {
    if (this->d < 4) {
      return 0;
    }

    if (this->d < 8) {
      return 1;
    }

    return 2;
  }
};

#endif