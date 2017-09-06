#include <uWS/uWS.h>
#include "json.hpp"
#include "PID.h"
#include <fstream>
#include "twiddle.h"
#include "Simulator.h"

using namespace std;

void RunPid() {
    //10.025,0.00461549,-4.0861
    PID pid_steering("steering");
    PID pid_throttle("throttle");

    const double desired_speed = 30;

    ofstream log;
    Simulator s;

    s.OnInitialize([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
        pid_steering.Init(10.025,0.00461549,-4.0861, measurement.cte);
        pid_throttle.Init(1.56807,0.00243957,-0.0972004, desired_speed - measurement.speed);

        log.open("data.log");
        s.SendReset(ws);
    });

    s.OnTelemetry([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
        ControlInput control;

        pid_steering.UpdateError(measurement.cte);
        control.steering = pid_steering.GetOutput();

        auto speed_cte = measurement.speed - desired_speed;
        pid_throttle.UpdateError(speed_cte);
        control.throttle = pid_throttle.GetOutput();

//        if (measurement.speed < 20)
//            control.throttle = 0.3;
//        else
//            control.throttle = 0;

        if (log.is_open()) {
            log << measurement << ", " << control.steering << ", " << control.throttle << endl;
        }

        s.SendControl(ws, control);
    });

    s.Run();
}


void RunSteeringTwiddle() {
    SteeringTwiddle t(0.002);
    t.Start();
}

void RunThrottleTwiddle() {
    ThrottleTwiddle t(0.002);
    t.Start();
}

int main()
{
    RunPid();
}
