#include <map>
#include <string>
#include <vector>

#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

const double STOP_COST = 0.8;
const double MAX_ACC = 5.0;
const double DELTA_T = 0.02;
const double SPEED_LIMIT = 50;
const double BUFFER_V = 2.0;
const double MPH_TO_MS = 0.447;
const double MS_TO_MPH = 1 / 0.447;
const int TRAJECTORY_LENGTH = 100;

std::map<std::string, std::vector<std::string>> STATES = {
    {"STOP", {"ACC", "STOP"}},
    {"ACC", {"CRUISE", "ACC", "DECC"}},
    {"DECC", {"CRUISE", "STOP", "DECC", "ACC"}},
    {"CRUISE", {"ACC", "DECC", "CRUISE"}},
};

#endif