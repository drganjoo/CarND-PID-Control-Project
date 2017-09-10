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
  SpeedController speed_controller(30, 30, 1.3);
  PIDSteering pid_steering(-0.09, 0, -10000);

  int iterations = 0;
  int stop_after_iterations = 4000;

  ostringstream file_name;
  file_name << "./log_" << chrono::system_clock::now().time_since_epoch().count() << ".csv";
  ofstream log;
  log.open(file_name.str());

  if (log.is_open()) {
    log << "CTE,Speed,Angle,c_throttle,c_dt_secs,p_error,i_error,d_error,steering,throttle" << endl;
  }

  chrono::system_clock::time_point last_check = chrono::system_clock::now();

  s.OnInitialize([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
    pid_steering.SetInitialCte(measurement);
    speed_controller.SetInitialCte(measurement);
  });

  s.OnTelemetry([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
    ControlInput control;

    iterations++;
    control.throttle = 0.5;
    control.steering = 0.0;
    s.SendControl(ws, control);

    return;


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

      ostringstream oss;
      cout << setw(6) << iterations << " CTE: "<< measurement.cte << "\tAngle: " << setw(6) << measurement.angle
           << "\tSpeed: " << measurement.speed << "\tD_Error_: " << pid_steering.GetDError()
           << "\tSTEERING: " << setw(6) << control.steering
           << "\tTHROTTLE: " << control.throttle << endl;
    }

//    if (log.is_open()) {
//      log << measurement.cte << "," << measurement.angle << "," << measurement.speed << ","
//          << measurement.c_throttle << "," << measurement.c_dt_secs << ","
//          << pid_steering.GetPError() << "," << pid_steering.GetIError() << ","
//          << pid_steering.GetDError() << "," << control.steering << ","
//          << control.throttle << endl;
//    }

    s.SendControl(ws, control);

    if (iterations > stop_after_iterations) {
      s.SendReset(ws);
      s.Stop();
    }
  });

  s.Run();

  cout << "Going out" << endl;
}

int main()
{
  //RunPid();
  //RunSteeringTwiddle();
  //RunThrottleTwiddle();
  ThrottleTest();
}
