#ifndef LOCALIZATION_H
#define LOCALIZATION_H

struct Localization {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  double prev_path_s;
  double prev_path_d;
};

#endif