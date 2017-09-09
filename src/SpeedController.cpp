//
// Created by fahad on 9/9/2017.
//

#include "SpeedController.h"
#include "Simulator.h"
#include "PID.h"
#include <fstream>
#include <memory>

using namespace std;

SpeedController::SpeedController(double desired_speed) {
  desired_speed_ = desired_speed;

  ifstream params;
  params.open("throttle_pid.txt");

  double kp, ki, kd;

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

  pid_throttle_ = unique_ptr<PIDThrottle>(new PIDThrottle(kp, ki, kd));
  cout << "Throttle params: " << kp << ", " << ki << ", " << kd << endl;
}

double SpeedController::GetThrottle(const TelemetryMessage &measurement) {
  SetDesiredSpeed(measurement);
  return pid_throttle_->GetOutput(measurement);
}


void SpeedController::SetDesiredSpeed(const TelemetryMessage &measurement) {
  const double MAX_SPEED = 70;
  const double BREAKING_SPEED = 20;

  static bool is_breaking = false;
  static int breaking_iterations = 0;

  double desired_speed;
  double cte_angle = rad2deg(measurement.cte);
//        const double angle_to_check = measurement.angle;
  const double angle_to_check = cte_angle;

  cout << "Angle to check: " << angle_to_check << endl;

  if (!is_breaking) {
    if (fabs(angle_to_check) > 5) {
      breaking_iterations = 0;
      is_breaking = true;
      desired_speed = BREAKING_SPEED;
    }
    else {
      desired_speed = MAX_SPEED ;
    }
  }
  else {
    if (fabs(angle_to_check) > 5) {
      breaking_iterations = 0;
      desired_speed = BREAKING_SPEED;
    }
    else if (breaking_iterations++ > 30) {
      // enough time has passed at this speed and the cte is within
      // acceptable range so lets accelerate again
      is_breaking = false;
      desired_speed = MAX_SPEED;
    }
    else {
      desired_speed = BREAKING_SPEED;
    }
  }

  pid_throttle_->SetDesiredSpeed(desired_speed);
}