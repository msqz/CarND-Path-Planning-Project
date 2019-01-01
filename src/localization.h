#ifndef LOCALIZATION_H
#define LOCALIZATION_H

struct Localization {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  
  //TODO unify with obstacle
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