#ifndef LOCALIZATION_H
#define LOCALIZATION_H
#include "constraints.h"

struct Localization {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;

  int get_lane() {
    if (0 <= this->d && this->d <= LANE_WIDTH) {
      return 0;
    }
    if (LANE_WIDTH <= this->d && this->d < 2.0 * LANE_WIDTH) {
      return 1;
    } 
    if (2.0 * LANE_WIDTH <= this->d && this->d < 3.0 * LANE_WIDTH) {
      return 2;
    }

    return -1;
  }
};

#endif
