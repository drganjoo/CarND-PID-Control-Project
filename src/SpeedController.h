//
// Created by fahad on 9/9/2017.
//

#ifndef PID_SPEEDCONTROLLER_H
#define PID_SPEEDCONTROLLER_H

#include "Simulator.h"

class SpeedController {
 public:
  SpeedController(double desired_speed);

  double GetThrottle(const TelemetryMessage &measurement);

 private:
  double desired_speed_;
  double last_cte_ = 0;
  double i_error_ = 0;
  double kp;
  double ki;
  double kd;
};

#endif //PID_SPEEDCONTROLLER_H
