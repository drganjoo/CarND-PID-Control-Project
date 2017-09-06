#include <uWS/uWS.h>
#include "json.hpp"
#include "PID.h"
#include <fstream>
#include "twiddle.h"
#include "Simulator.h"

using namespace std;

void RunPid() {
    PID pid_steering("steering");
    ofstream log;

    Simulator s;

    s.OnInitialize([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
        pid_steering.Init(0.1, 0.0, 0.0, measurement.cte);
        log.open("data.log");
    });

    s.OnTelemetry([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
        ControlInput control;

        pid_steering.UpdateError(measurement.cte);
        control.steering = pid_steering.GetOutput();

        if (measurement.speed < 20)
            control.throttle = 0.3;
        else
            control.throttle = 0;

        if (log.is_open()) {
            log << measurement << ", " << control.steering << ", " << control.throttle << endl;
        }

        s.SendControl(ws, control);
    });

    s.Run();
}

int main()
{
    Twiddle t(0.002);
    t.Start();
}
