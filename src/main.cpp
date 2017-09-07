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
    SteeringTwiddle t(0.002);
    t.Start();
}

void RunThrottleTwiddle() {
    ThrottleTwiddle t(0.002);
    t.Start();
}

void ThrottleTest(){
    Simulator s;
    PID pid_throttle;
    PID pid_steering;

    double desired_speed = 20;
    int iterations = 0;
    int calc_after_iterations_ = 10;
    int stop_after_iterations = 2000;

    s.OnInitialize([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
        //pid_throttle.Init(-0.1, -0.0054, 0, desired_speed - measurement.speed);
        pid_throttle.Init(-0.1, -0.0054, 0, desired_speed - measurement.speed);
        pid_steering.Init(0.08, 0, 0, measurement.cte);
    });

    s.OnTelemetry([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
        ControlInput control;

        iterations++;

        auto speed_cte = desired_speed - measurement.speed;
        pid_throttle.UpdateError(speed_cte, iterations > calc_after_iterations_);
        control.throttle = pid_throttle.GetOutput();

        pid_steering.UpdateError(measurement.cte, iterations > calc_after_iterations_);
        control.steering = pid_steering.GetOutput();

        cout << iterations << ": Measurement --> cte:" << measurement.cte
             << ", speed: " << measurement.speed << ", angle: " << measurement.angle
             << " Control ->  steering: " << control.steering
             << " throttle: " << control.throttle << endl;

        s.SendControl(ws, control);

        if (iterations > stop_after_iterations) {
            s.Stop();
        }
    });

    s.Run();
}

int main()
{
    //RunPid();
    //RunSteeringTwiddle();

    ThrottleTest();
}
