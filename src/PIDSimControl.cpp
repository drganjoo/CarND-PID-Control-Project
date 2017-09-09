//
// Created by fahad on 9/7/2017.
//

#include "PIDSimControl.h"
#include <fstream>

using namespace std;

PIDSimControl::PIDSimControl() :
  pid_steering(0.077, 0.00561549, -2.0),
  pid_throttle(1.56807, 0.00243957, -0.0972004)
{

}

void PIDSimControl::Run()
{
  ofstream log;
  int iterations = 0;

  s.OnInitialize([&](uWS::WebSocket <uWS::SERVER> &ws, const TelemetryMessage &measurement) {
    pid_steering.SetInitialCte(measurement);
    pid_throttle.SetInitialCte(measurement);

    log.open("data.log");
    s.SendReset(ws);
  });

  s.OnTelemetry([&](uWS::WebSocket <uWS::SERVER> &ws, const TelemetryMessage &measurement) {
    ControlInput control;

    control.steering = pid_steering.GetOutput(measurement);
    control.throttle = GetThrottle(measurement);

    s.SendControl(ws, control);

    if (log.is_open()) {
      log << measurement << ", " << control.steering << ", " << control.throttle << endl;
    }

    iterations++;

//        cout << iterations << " -> Error: " << pid_steering_.TotalError()
//             << "\tAngle: " << measurement.angle
//             << "\tSpeed: " << measurement.speed
//             << "\tDelta Angle: " << pid_steering_.GetDError() << " , " << cte_angle
//             << endl;
  });

  s.Run();
}

void PIDSimControl::SetDesiredSpeed(const TelemetryMessage &measurement) {
  const double MAX_SPEED = 100;
  const double BREAKING_SPEED = 20;

  static bool is_breaking = false;
  static int breaking_iterations = 0;

  double desired_speed;
  double cte_angle = rad2deg(measurement.cte);
//        const double angle_to_check = measurement.angle;
  const double angle_to_check = cte_angle;

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

  pid_throttle.SetDesiredSpeed(desired_speed);
}

double PIDSimControl::GetThrottle(const TelemetryMessage &measurement) {
  SetDesiredSpeed(measurement);
  return pid_throttle.GetOutput(measurement);
}
