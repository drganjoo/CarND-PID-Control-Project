#include <uWS/uWS.h>
#include "json.hpp"
#include "PID.h"
#include <fstream>
#include "twiddle.h"
#include "Simulator.h"
#include "SpeedController.h"

using namespace std;

void RunPid() {
}

void RunSteeringTwiddle() {
  Twiddle<SteeringTwiddle> t(0.00002, TwiddleParams({-0.07,0,0}, {0.02, 0, 0}));
  t.Run(2);
}

void RunThrottleTwiddle() {
  Twiddle<ThrottleTwiddle> t(0.0002, TwiddleParams({0.08,0,0}, {0.02, 0, 0}));
  t.Run();
}

void ThrottleTest(){
  Simulator s;
  SpeedController speed_controller(30);
  PIDSteering pid_steering(-1, 0, 0);

  int iterations = 0;
  int stop_after_iterations = 4000;
//  bool speed_slow = false;

  chrono::system_clock::time_point last_check = chrono::system_clock::now();

  s.OnInitialize([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
    pid_steering.SetInitialCte(measurement);
    speed_controller.SetInitialCte(measurement);
  });

  s.OnTelemetry([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
    ControlInput control;

    iterations++;

//    if (iterations < 1000) {
//      speed_controller.SetMaxSpeed(100.0);
//    }
//    else {
//      if (!speed_slow) {
//        speed_slow = true;
//        cout << "Reducing speed" << endl;
//      }
//    }

    control.throttle = speed_controller.GetOutput(measurement);
    control.steering  = pid_steering.GetOutput(measurement);

    auto now = chrono::system_clock::now();
    auto diff = chrono::duration_cast<chrono::milliseconds>(now - last_check);

    if (diff.count() > 300.0) {
      last_check = now;

      cout << setw(6) << iterations << " CTE: "<< measurement.cte << "\tAngle: " << measurement.angle
           << "\tSpeed: " << measurement.speed
           << "\tSTEERING: " << control.steering
           << "\tTHROTTLE: " << control.throttle << endl;
    }

    s.SendControl(ws, control);

    if (iterations > stop_after_iterations) {
      s.SendReset(ws);
      s.Stop();
    }
  });

  s.Run();
}

int main()
{
  //RunPid();
  //RunSteeringTwiddle();
  //RunThrottleTwiddle();
  ThrottleTest();
}
