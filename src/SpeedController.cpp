//
// Created by fahad on 9/9/2017.
//

#include "SpeedController.h"
#include "Simulator.h"
#include "PID.h"
#include <fstream>
#include <memory>

using namespace std;

SpeedController::SpeedController(double max_speed) :
  SpeedController(max_speed, 20.0, 1.3)
{
}

SpeedController::SpeedController(double max_speed, double breaking_speed, double breaking_cte) :
  breaking_cte_(breaking_cte),
  breaking_speed_(breaking_speed),
  min_good_secs_(3000),
  pid_throttle_(0.8, 0, 0)
{
  max_speed_ = max_speed;
  pid_throttle_.SetDesiredSpeed(max_speed_);
}

double SpeedController::GetOutput(const TelemetryMessage &measurement) {
  double speed = GetSpeedForAngle(measurement);
  pid_throttle_.SetDesiredSpeed(speed);
  return pid_throttle_.GetOutput(measurement);
}


double SpeedController::GetSpeedForAngle(const TelemetryMessage &measurement) {
  double desired_speed;
  double cte_angle = measurement.cte;
//        const double angle_to_check = measurement.angle;
  const double angle_to_check = cte_angle;

  if (!state_.is_breaking) {
    if (fabs(angle_to_check) > breaking_cte_) {
      state_.steering_ok_for_ms = 0;
      state_.is_breaking = true;
      desired_speed = breaking_speed_;

      cout << "BREAK!!!!! CTE = " << measurement.cte << " Angle = " << measurement.angle << " Formula: "
           << angle_to_check << " > " << breaking_cte_ << " therefore break" << endl;
    }
    else {
      desired_speed = max_speed_;
    }
  }
  else {
    desired_speed = breaking_speed_;
    state_.steering_ok_for_ms += measurement.c_dt_secs;

    if (fabs(angle_to_check) > breaking_cte_) {
      state_.steering_ok_for_ms = 0;
    }
    else if (state_.steering_ok_for_ms > min_good_secs_) {
      // enough time has passed at this speed and the cte is within
      // acceptable range so lets accelerate again
      state_.is_breaking = false;
      desired_speed = max_speed_;

      cout << "Speeding!!!!! We have been good for " << state_.steering_ok_for_ms
           << " milliseconds, CTE = " << measurement.cte << " Angle = " << measurement.angle << " Formula: "
           << angle_to_check << " < " << breaking_cte_ << " therefore break" << endl;
    }
  }

  return desired_speed;
}
