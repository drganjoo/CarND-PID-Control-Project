//
// Created by fahad on 9/7/2017.
//

#ifndef PID_PIDSIMCONTROL_H
#define PID_PIDSIMCONTROL_H


#include <string>
#include "Simulator.h"
#include "PID.h"

class PIDSimControl {
 public:
  PIDSimControl();
  void Run();
  void SetDesiredSpeed(const TelemetryMessage &measurement);
  double GetThrottle(const TelemetryMessage &measurement);

 private:
  PIDSteering pid_steering;
  PIDThrottle pid_throttle;
  Simulator s;
};


#endif //PID_PIDSIMCONTROL_H
