#pragma once

#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "helpers.h"

struct TelemetryMessage {
    double cte;
    double speed;
    double angle;

    friend std::ostream& operator << (std::ostream &os, const TelemetryMessage &m);
};

struct ControlInput {
    double steering;
    double throttle;

    friend std::ostream& operator <<(std::ostream &os, const ControlInput &c);
};

class Simulator
{
public:
    typedef std::function<void(uWS::WebSocket<uWS::SERVER> &, const TelemetryMessage &)> TelemetryCallbackFunc;

    Simulator();
    ~Simulator() = default;

    void OnInitialize(TelemetryCallbackFunc initialize_func) {
      initialize_ = std::move(initialize_func);
    }

    void OnTelemetry(TelemetryCallbackFunc telemetry_func) {
      telemetry_ = std::move(telemetry_func);
    }

    void Run();
    void Stop();
    void SendManualMode(uWS::WebSocket<uWS::SERVER> &ws);
    void SendReset(uWS::WebSocket<uWS::SERVER> &ws);
    void SendControl(uWS::WebSocket<uWS::SERVER> &ws, const ControlInput &control);

private:
    int Parse(char *data, size_t length, TelemetryMessage *tm);
    bool IsValidData(const std::string &line);
    std::string GetJson(const std::string &line);

    TelemetryCallbackFunc initialize_;
    TelemetryCallbackFunc telemetry_;

    uWS::Hub hub_;
};