#include <uWS/uWS.h>
#include "json.hpp"
#include "PID.h"
#include <fstream>
#include "twiddle.h"
#include "Simulator.h"

using namespace std;

void RunPid() {
}


void RunSteeringTwiddle() {
  SteeringTwiddle t(0.00002);
  t.StartCheckBoth();
}

void RunThrottleTwiddle() {
  ThrottleTwiddle t(0.00002, 20.0);
  t.StartCheckBoth();
}

void ThrottleTest(){
  Simulator s;
  PIDThrottle pid_throttle(20);
  PID pid_steering;

  int iterations = 0;
  int stop_after_iterations = 2000;

  s.OnInitialize([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
    //pid_throttle.Init(-0.1, -0.0054, 0, desired_speed - measurement.speed);
    //pid_throttle.Init(-0.1, -0.003, 0, desired_speed - measurement.speed);
    pid_throttle.Init(-0.08, 0, 0, measurement);
    pid_steering.Init(0.08, 0, 0, measurement.cte);
  });

  s.OnTelemetry([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
    ControlInput control;

    iterations++;

    pid_throttle.UpdateError(measurement, true);
    control.throttle = pid_throttle.GetOutput();

    pid_steering.UpdateError(measurement.cte, true);
    control.steering = pid_steering.GetOutput();

    cout << iterations << ": Measurement --> cte:" << measurement.cte
         << ", speed: " << measurement.speed << ", angle: " << measurement.angle
         << " Control ->  steering: " << control.steering
         << " throttle: " << control.throttle << endl;

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
