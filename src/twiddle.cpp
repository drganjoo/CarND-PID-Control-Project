#include <uWS/uWS.h>
#include "twiddle.h"
#include "Simulator.h"
#include <fstream>
#include <functional>
#include <sstream>
#include <iterator>

using namespace std;
using namespace std::placeholders;



/*----------------------------------------------------------------------------------------*/

SteeringTwiddle::SteeringTwiddle(double kp, double ki, double kd) :
    pid_steering_(kp, ki, kd),
    speed_controller_(40.0)
{
}

double SteeringTwiddle::Run() {
  sim_.OnInitialize([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
    pid_steering_.SetInitialCte(measurement);
    speed_controller_.SetInitialCte(measurement);

    iterations_ = 0;
  });

  sim_.OnTelemetry([this](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
                     iterations_++;

                     ControlInput control;
                     control.throttle = speed_controller_.GetOutput(measurement);
                     control.steering = pid_steering_.GetOutput(measurement);

                     sim_.SendControl(ws, control);

                     if (iterations_ > stop_after_iterations_) {
                       sim_.Stop();
                     }
                     else if (iterations_ < calc_after_iterations_)
                       pid_steering_.ResetTotalError();
                   }
  );

  sim_.Run();

  return pid_steering_.GetAccumError();
}

/*----------------------------------------------------------------------------------------*/

ThrottleTwiddle::ThrottleTwiddle(double kp, double ki, double kd) :
    pid_steering_(0.08, 0, 0),
    pid_throttle_(kp, ki, kd)
{
}

double ThrottleTwiddle::Run() {
  double error_to_return = 0.0;

  sim_.OnInitialize([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
    pid_steering_.SetInitialCte(measurement);
    pid_throttle_.SetInitialCte(measurement);

    iterations_ = 0;
  });

  sim_.OnTelemetry([this, &error_to_return](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
                     if (IsCarGoingInReverse(measurement)) {
                       cout << "stopping!!! car is going in reverse" << endl;
                       error_to_return = INT_MAX;
                       sim_.Stop();
                     }
                     else {
                       iterations_++;

                       ControlInput control;
                       control.throttle = pid_throttle_.GetOutput(measurement);
                       control.steering = pid_steering_.GetOutput(measurement);

                       sim_.SendControl(ws, control);

                       if (iterations_ > stop_after_iterations_) {
                         error_to_return = pid_throttle_.GetAccumError();
                         sim_.Stop();
                       } else if (iterations_ < calc_after_iterations_)
                         pid_steering_.ResetTotalError();
                     }
                   }
  );

  sim_.Run();

  return error_to_return;
}

bool ThrottleTwiddle::IsCarGoingInReverse(const TelemetryMessage &measurement) {
  static double last_speed = measurement.speed;
  static double speed_integral = 0;

  bool reverse = false;

  if (measurement.c_throttle < 0) {
    double speed_derivative = measurement.speed - last_speed;
    last_speed = measurement.speed;

    if (speed_derivative >= 0) {
      speed_integral += speed_derivative * 0.1;

      // hmmm is the car going in reverse now since the throttle is -ve
      // but speed is increasing. Lets see for the next few seconds to make sure this
      // happens
      if (speed_integral >= 5) {

        // for sure it is going in reverse, lets stop and return a big number from total error
        reverse = true;
      }
    }
    else {
      speed_integral = 0;
    }
  } else {
    speed_integral = 0;
  }

  return reverse;
}

