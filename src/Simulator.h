#pragma once

#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "helpers.h"
#include <chrono>

struct TelemetryMessage {
  double cte;
  double speed;
  double angle;
  double c_throttle;
  double c_dt_secs;

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
    initialize_fp = std::move(initialize_func);
  }

  void OnTelemetry(TelemetryCallbackFunc telemetry_func) {
    telemetry_fn = std::move(telemetry_func);
  }

  void Run();
  void Stop();
  void SendManualMode(uWS::WebSocket<uWS::SERVER> &ws);
  void SendReset(uWS::WebSocket<uWS::SERVER> &ws);
  void SendControl(uWS::WebSocket<uWS::SERVER> &ws, const ControlInput &control);

 protected:
  void SendResetOnMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode);
  void InitialOnMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode);
  void OnMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode);

 private:
  int Parse(char *data, size_t length, TelemetryMessage *measurement);
  bool IsValidData(const std::string &line);
  std::string GetJson(const std::string &line);

  TelemetryCallbackFunc initialize_fp;
  TelemetryCallbackFunc telemetry_fn;

  uWS::Hub hub_;
  ControlInput last_control_;
  unsigned int settle_down_iterations_ = 0;
  std::chrono::system_clock::time_point last_call_;
  //std::chrono::milliseconds last_call_;
  unsigned int frames_per_sec_ = 0;
};