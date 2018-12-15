#include <map>
#include <string>
#include <vector>

#ifndef CAR_H
#define CAR_H

struct Trajectory {
  std::vector<double> s;
  std::vector<double> d;

  double get_velocity();
};

struct Localization {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
};

struct SensorFusion {
  std::string id;
  double x;
  double y;
  double x_velocity;
  double y_velocity;
  double s;
  double d;
};

class Car {
  std::string state;
  Localization localization;
  Trajectory build_trajectory(std::string state);

 public:
  Car();
  Trajectory get_trajectory();
  void set_localization(Localization localization);
  std::string get_state();
};

#endif