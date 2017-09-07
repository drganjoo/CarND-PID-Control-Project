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

int main()
{
    //RunPid();
    RunSteeringTwiddle();
}
