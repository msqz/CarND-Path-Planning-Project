#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <map>
#include <string>
#include <vector>

const double STOP_COST = 0.8;
const double MAX_ACC = 10.0;
const double DELTA_T = 0.02;
const double SPEED_LIMIT = 100.0;
const double BUFFER_V = 4.0;
const double MPH_TO_MS = 0.447;
const double MS_TO_MPH = 1 / 0.447;
const int PATH_LENGTH = 100;
const double TRACK_LENGTH = 6945.554;
const double LANE_WIDTH = 4.0;
const double BRAKING_DECC = 0.9 * MAX_ACC;
const double FRONT_DISTANCE = 4.0;

#endif /* CONSTRAINTS_H */