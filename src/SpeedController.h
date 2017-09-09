//
// Created by fahad on 9/9/2017.
//

#ifndef PID_SPEEDCONTROLLER_H
#define PID_SPEEDCONTROLLER_H

#include "Simulator.h"
#include "PID.h"
#include <memory>

class SpeedController {
 public:
  SpeedController(double desired_speed);

  double GetThrottle(const TelemetryMessage &measurement);
  void SetInitialCte(const TelemetryMessage &measurement) {
    pid_throttle_->SetInitialCte(measurement);
  }

 protected:
  void SetDesiredSpeed(const TelemetryMessage &measurement);

 private:
  double desired_speed_;
  std::unique_ptr<PIDThrottle> pid_throttle_;

//  double last_cte_ = 0;
//  double i_error_ = 0;
//  double kp;
//  double ki;
//  double kd;
};

#endif //PID_SPEEDCONTROLLER_H
