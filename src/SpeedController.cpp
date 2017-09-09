//
// Created by fahad on 9/9/2017.
//

#include "SpeedController.h"
#include "Simulator.h"
#include <fstream>

using namespace std;

SpeedController::SpeedController(double desired_speed) {
  desired_speed_ = desired_speed;

  ifstream params;
  params.open("throttle_pid.txt");

  if (params.is_open()){
    params >> kp >> ki >> kd;
    params.close();
  }
  else {
    cout << "Could not open throttle pid file" << endl;
    kp = 0.8;
    ki = 0.0;
    kd = 0.0;
  }

  cout << "Throttle params: " << kp << ", " << ki << ", " << kd << endl;
}

double SpeedController::GetThrottle(const TelemetryMessage &measurement) {
  double throttle;

  if (fabs(measurement.angle) > 1.2) {
    desired_speed_ = 20;
  }

  double speed_cte = desired_speed_ - measurement.speed;

  double p_error = speed_cte;
  double d_error = speed_cte - last_cte_;
  i_error_ += speed_cte;
  last_cte_ = d_error;

  throttle = p_error * kp + i_error_ * ki + d_error * kd;

  if (throttle > 1)
    throttle = 1;
  else if (throttle < -1)
    throttle = -1;

  if (measurement.speed > 35) {
    cout << "throttle " << throttle << endl;
  }

  // slow down on turns
//  if (fabs(measurement.angle) > 1.2) {
//    if (measurement.speed > 50) {
//      throttle = -1;
//    }
//    else if (measurement.speed > 35) {
//      throttle = -0.5;
//    }
//    else if (measurement.speed > 25) {
//      throttle = -0.2;
//    }
//    else {
//      throttle = 0.1;
//    }
//  }

  return throttle;
}
