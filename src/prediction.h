#ifndef PREDICTION_H
#define PREDICTION_H

#include <vector>
#include "localization.h"

struct Car {
  //x, y, velocity etc
};

struct Prediction {
  std::vector<Car> cars;
  Localization localization;
};

#endif