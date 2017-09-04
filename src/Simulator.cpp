#include "Simulator.h"
#include <string>
#include "helpers.h"
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

Simulator::Simulator()
{
	initialize_ = nullptr;
	telemetry_ = nullptr;
}

Simulator::~Simulator()
{
}

void Simulator::Run()
{
	uWS::Hub h;

	h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		initialize_(ws);

		// from next time call this other lambda
		h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
			data[length] = 0;
			string line = data;

			if (IsValidData(line)) {
				string jsonStr = GetJson(line);

				if (jsonStr.length() == 0)
					SendManualMode(ws);
				else {
					auto jsonObj = json::parse(jsonStr);
					auto wsEvent = jsonObj[0].get<std::string>();

					if (wsEvent == "telemetry") {
						TelemetryMessage measurement;

						measurement.cte = stod(jsonObj[1]["cte"].get<string>());
						measurement.speed = stod(jsonObj[1]["speed"].get<string>());
						measurement.angle = stod(jsonObj[1]["steering_angle"].get<string>());


						telemetry_(ws, measurement);
					}
				}
			}
		});
	});

	h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		cout << "Connected!!!" << endl;
	});

	h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		cout << "Disconnected" << endl;
	});

	int port = 4567;
	if (h.listen("127.0.0.1", port))
		cout << "Listening on port " << port << endl;
	else {
		cerr << "Failed to listen to port" << endl;
		exit(-1);
	}

	h.run();
}

void Simulator::SendManualMode(uWS::WebSocket<uWS::SERVER> &ws)
{
	std::string msg = "42[\"manual\",{}]";
	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void Simulator::SendReset(uWS::WebSocket<uWS::SERVER> &ws)
{
	std::string msg = "42[\"reset\",{}]";
	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

bool Simulator::IsValidData(const string &line)
{
	return line.length() > 2 && line[0] == '4' && line[1] == '2';
}

std::string Simulator::GetJson(const string &line)
{
	if (line.find("null") != string::npos)
		return "";

	auto start = line.find_first_of("[");
	auto end = line.find_last_of("]");

	if (start == string::npos || end == string::npos)
		return "";

	return line.substr(start, end - start + 1);
}

