#pragma once

#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "helpers.h"

class Simulator
{
public:
	typedef std::function<void(uWS::WebSocket<uWS::SERVER> &)> CallbackFunc;
	typedef std::function<void(uWS::WebSocket<uWS::SERVER> &, const TelemetryMessage &)> JsonCallbackFunc;

	Simulator();
	~Simulator();

	void OnInitialize(CallbackFunc initialize_func) {
		initialize_ = initialize_func;
	}

	void OnTelemetry(JsonCallbackFunc telemetry_func) {
		telemetry_ = telemetry_func;
	}

	void Run();
	void SendManualMode(uWS::WebSocket<uWS::SERVER> &ws);
	void SendReset(uWS::WebSocket<uWS::SERVER> &ws);

private:
	bool IsValidData(const std::string &line);
	std::string GetJson(const std::string &line);

	CallbackFunc initialize_;
	JsonCallbackFunc telemetry_;
};