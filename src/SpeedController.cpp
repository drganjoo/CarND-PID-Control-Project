//
// Created by fahad on 9/9/2017.
//

#include "SpeedController.h"
#include "Simulator.h"
#include "PID.h"
#include <fstream>
#include <memory>

using namespace std;

SpeedController::SpeedController(double desired_speed) :
    max_angle_(1.3),
    breaking_speed_(20),
    min_good_secs_(3000)
{
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

  max_speed_ = desired_speed;

  pid_throttle_ = unique_ptr<PIDThrottle>(new PIDThrottle(kp, ki, kd));
  pid_throttle_->SetDesiredSpeed(max_speed_);

  cout << "Throttle params: " << kp << ", " << ki << ", " << kd << endl;
}

double SpeedController::GetOutput(const TelemetryMessage &measurement) {
  SetDesiredSpeed(measurement);
  return pid_throttle_->GetOutput(measurement);
}


void SpeedController::SetDesiredSpeed(const TelemetryMessage &measurement) {

  double desired_speed;
  double cte_angle = measurement.cte;
//        const double angle_to_check = measurement.angle;
  const double angle_to_check = cte_angle;

  if (!state_.is_breaking) {
    if (fabs(angle_to_check) > max_angle_) {
      state_.steering_ok_for_ms = 0;
      state_.is_breaking = true;
      desired_speed = breaking_speed_;

      cout << "BREAK!!!!! CTE = " << measurement.cte << " Angle = " << measurement.angle << " Formula: "
           << angle_to_check << " > " << max_angle_ << " therefore break" << endl;
    }
    else {
      desired_speed = max_speed_;
    }
  }
  else {
    desired_speed = breaking_speed_;
    state_.steering_ok_for_ms += measurement.dt_secs;

    if (fabs(angle_to_check) > max_angle_) {
      state_.steering_ok_for_ms = 0;
    }
    else if (state_.steering_ok_for_ms > min_good_secs_) {
      // enough time has passed at this speed and the cte is within
      // acceptable range so lets accelerate again
      state_.is_breaking = false;
      desired_speed = max_speed_;

      cout << "Speeding!!!!! We have been good for " << state_.steering_ok_for_ms
           << " milliseconds, CTE = " << measurement.cte << " Angle = " << measurement.angle << " Formula: "
           << angle_to_check << " < " << max_angle_ << " therefore break" << endl;
    }
  }

  pid_throttle_->SetDesiredSpeed(desired_speed);
}