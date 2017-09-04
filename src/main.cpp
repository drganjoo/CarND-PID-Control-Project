#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <string>
#include "helpers.h"

using namespace std;
using json = nlohmann::json;

uWS::Hub h;
tuple<double, double> (*fp)(PID &pid, uWS::WebSocket<uWS::SERVER> &ws, const Measurement &m) = nullptr;


bool IsValidData(const string &line) {
  return line.length() > 2 && line[0] == '4' && line[1] == '2';
}

string GetJson(const string &line) {
  if (line.find("null") != string::npos)
    return "";

  auto start = line.find_first_of("[");
  auto end = line.find_last_of("]");

  if (start == string::npos || end == string::npos)
    return "";

  return line.substr(start, end - start + 1);
}

void SendManualMode(uWS::WebSocket<uWS::SERVER> &ws){
  std::string msg = "42[\"manual\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void SendReset(uWS::WebSocket<uWS::SERVER> &ws){
  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

tuple<double, double> Control(PID &pid, uWS::WebSocket<uWS::SERVER> &ws, const Measurement &m) {
  pid.UpdateError(m.cte);
  double steer_value = pid.GetOutput();

  /*
  * TODO: Calcuate steering value here, remember the steering value is
  * [-1, 1].
  * NOTE: Feel free to play around with the throttle and speed. Maybe use
  * another PID controller to control the speed!
  */

  return tuple<double, double>(steer_value, 0.3);
}

tuple<double, double> Initialize(PID &pid, uWS::WebSocket<uWS::SERVER> &ws, const Measurement &m){
  pid.Init(0.1, 4.0, 0.03, m.cte);
  fp = Control;

  return {0, 0.3};
};


int main()
{
  PID pid;

  fp = Initialize;

  // TODO: Initialize the pid variable.

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
            Measurement measurement;

            measurement.cte = stod(jsonObj[1]["cte"].get<string>());
            measurement.speed = stod(jsonObj[1]["speed"].get<string>());
            measurement.angle = stod(jsonObj[1]["steering_angle"].get<string>());

//            cout << "CTE: " << measurement.cte << ", Speed:" <<
//                 measurement.speed << ", Angle:" << measurement.angle <<  endl;

            tuple<double, double> control_vars = (*fp)(pid, ws, measurement);
//            tuple<double, double> control_vars = {-1, 0.3};

            double steer_value =  std::get<0>(control_vars);
            double speed = std::get<1>(control_vars);

            // DEBUG
            std::cout << "CTE: " << measurement.cte << " Steering Value: " << steer_value << std::endl;

            json msgJson = {
                    {"steering_angle", steer_value},
                    {"throttle", speed}
            };

            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;

            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      }
      });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
