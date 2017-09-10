//
// Created by fahad on 9/9/2017.
//

#ifndef PID_SPEEDCONTROLLER_H
#define PID_SPEEDCONTROLLER_H

#include "Simulator.h"
#include "PID.h"
#include <memory>


struct CarState {
  bool is_breaking;
  double steering_ok_for_ms;

  CarState() {
    is_breaking = 0;
    steering_ok_for_ms = 0;
  }
};

class SpeedController {
 public:
  SpeedController(double desired_speed);

  double GetOutput(const TelemetryMessage &measurement);
  void SetInitialCte(const TelemetryMessage &measurement) {
    pid_throttle_->SetInitialCte(measurement);
  }

 protected:
  void SetDesiredSpeed(const TelemetryMessage &measurement);

 private:
  const double max_angle_;
  const double breaking_speed_;
  const double min_good_secs_;

  double max_speed_;

  CarState state_;

  std::unique_ptr<PIDThrottle> pid_throttle_;
};

#endif //PID_SPEEDCONTROLLER_H
