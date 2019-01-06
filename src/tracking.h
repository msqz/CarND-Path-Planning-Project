#ifndef PREDICTION_H
#define PREDICTION_H

#include <math.h>
#include <chrono>
#include <map>
#include <vector>
#include "constraints.h"
#include "obstacle.h"
// keep last 3 measurements for each visible obstacle
// keep them all (for now), i.e. no removing of obstacle from the map

int RELIABLE_LENGTH = 20;

struct Prediction {
  int id;
  double s;
  double d;
  double s_original;
  double d_original;
};

struct TrackingRecord {
  Obstacle obstacle;
  int timestamp;
};

struct Tracking {
  std::map<int, std::vector<TrackingRecord>> history;

  void add(Obstacle obstacle) {
    auto now = std::chrono::system_clock::now();
    int timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    if (history.find(obstacle.id) == history.end()) {
      history[obstacle.id] = {};
    } else if (history[obstacle.id].size() == RELIABLE_LENGTH) {
      history[obstacle.id].erase(history[obstacle.id].begin());
    }

    history[obstacle.id].push_back((TrackingRecord){
        .obstacle = obstacle,
        .timestamp = timestamp,
    });
  }

  std::vector<Prediction> predict(double t = DELTA_T * PATH_LENGTH) {
    std::vector<Prediction> predictions;
    for (auto pair : history) {
      std::vector<TrackingRecord> tracking = pair.second;
      Prediction prediction{
          .id = pair.first,
      };
      // prediction.s = tracking.back().obstacle.s;
      // prediction.d = tracking.back().obstacle.d;
      // prediction.s_original = prediction.s;
      // prediction.d_original = prediction.d;
      // predictions.push_back(prediction);
      // continue;

      if (tracking.size() < RELIABLE_LENGTH) {
        // Not enough data to predict anything
        prediction.s = tracking[0].obstacle.s;
        prediction.d = tracking[0].obstacle.d;
        prediction.s_original = prediction.s;
        prediction.d_original = prediction.d;
        predictions.push_back(prediction);
        // std::cout << "  id: " << tracking[0].obstacle.id
        //           << " s: " << tracking[0].obstacle.s
        //           << " d: " << tracking[0].obstacle.d
        //           << " dt: 0"
        //           << "\n";
      } else {
        // Can predict position based on velocity
        double dt = (tracking.back().timestamp - tracking.front().timestamp) / 1000;
        double v_s = (tracking.back().obstacle.s - tracking.front().obstacle.s) / dt;        
        double v_d = (tracking.back().obstacle.d - tracking.front().obstacle.d) / dt;        

        prediction.s = tracking.back().obstacle.s + (v_s * t);
        prediction.d = tracking.back().obstacle.d + (v_d * t);
        prediction.s_original = tracking.back().obstacle.s;
        prediction.d_original = tracking.back().obstacle.d;
        predictions.push_back(prediction);

        // std::cout << "  id: " << tracking[0].obstacle.id
        //           << " s: " << prediction.s
        //           << " d: " << prediction.d
        //           << " v_s: " << v_s
        //           << " v_d: " << v_d
        //           << " dt: " << dt
        //           << "\n";
      // } else {
      //   // Can predict position pased on velocity and acceleration
      //   double dt_0 = (tracking[1].timestamp - tracking[0].timestamp) / 1000.0;
      //   double dt_1 = (tracking[2].timestamp - tracking[1].timestamp) / 1000.0;
      //   double v_s_0 = (tracking[1].obstacle.s - tracking[0].obstacle.s) / dt_0;
      //   double v_s_1 = (tracking[2].obstacle.s - tracking[1].obstacle.s) / dt_1;
      //   double v_d_0 = (tracking[1].obstacle.d - tracking[0].obstacle.d) / dt_0;
      //   double v_d_1 = (tracking[2].obstacle.d - tracking[1].obstacle.d) / dt_1;
      //   double a_s = v_s_1 - v_s_0 / dt_1;
      //   double a_d = v_d_1 - v_d_0 / dt_1;

      //   prediction.s = tracking[2].obstacle.s + (v_s_1 * t) + (a_s * pow(t, 2) / 2);
      //   prediction.d = tracking[2].obstacle.d + (v_d_1 * t) + (a_d * pow(t, 2) / 2);
      //   predictions.push_back(prediction);
      }
    }

    return predictions;
  }
};

#endif
