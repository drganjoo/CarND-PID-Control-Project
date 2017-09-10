#include "Simulator.h"
#include "json.hpp"
#include <chrono>
#include <ctime>

using namespace std;
using namespace std::placeholders;
using namespace std::chrono;
using json = nlohmann::json;

std::ostream& operator <<(std::ostream &os, const TelemetryMessage &m) {
  os << m.cte << ", " << m.angle << ", " << m.speed;
  return os;
}

std::ostream& operator <<(std::ostream &os, const ControlInput &c) {
  os << c.steering << ", " << c.throttle;
  return os;
}

Simulator::Simulator()
{
  initialize_fp = nullptr;
  telemetry_fn = nullptr;
  last_control_.steering = 0;
  last_control_.throttle = 0;
}


int Simulator::Parse(char *data, size_t length, TelemetryMessage *measurement) {
  static chrono::system_clock::time_point last_check = system_clock::now();
  static unsigned int total_calls = 0;

  data[length] = 0;
  string line = data;
  int ret_code = -1;

  if (IsValidData(line)) {
    string jsonStr = GetJson(line);

    if (jsonStr.length() == 0)
      ret_code = 0;
    else {
      auto jsonObj = json::parse(jsonStr);
      auto wsEvent = jsonObj[0].get<std::string>();

      if (wsEvent == "telemetry") {
        total_calls++;

        measurement->cte = stod(jsonObj[1]["cte"].get<string>());
        measurement->speed = stod(jsonObj[1]["speed"].get<string>());
        measurement->angle = stod(jsonObj[1]["steering_angle"].get<string>());

        // simulator is not sending us the throttle back :( it is always 0
        // but we need this for checking if the car is going in reverse, hence
        // we use our last_control variable instead of the JSON
        //measurement->c_throttle = stod(jsonObj[1]["throttle"].get<string>());
        measurement->c_throttle = last_control_.throttle;

        //milliseconds now  = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        //measurement->c_dt_secs = (now.count() - last_call_.count()) / 1000.0;

        auto now = system_clock::now();
        measurement->c_dt_secs = duration_cast<milliseconds>(now - last_call_).count();
        measurement->c_dt_secs /= 1000.0;
        last_call_ = now;

//        cout << "DT_Secs: " << measurement->c_dt_secs << endl;

        if (duration_cast<milliseconds>(now - last_check).count() >= 1000) {
//          cout << "Frames / Sec: " << frames_per_sec_ << endl;

          last_check = now;
          frames_per_sec_ = total_calls;
          total_calls = 0;

//          cout << frames_per_sec_ << endl;
        }
        ret_code = 1;
      }
    }
  }

  return ret_code;
}

void Simulator::SendResetOnMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  hub_.onMessage(std::bind(&Simulator::InitialOnMessage, this, _1, _2, _3, _4));
  SendReset(ws);

  last_control_.steering = 0;
  last_control_.throttle = 0;
  settle_down_iterations_ = 0;
  last_call_  = system_clock::now();
}


void Simulator::InitialOnMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  TelemetryMessage measurement;

  auto status = Parse(data, length, &measurement);
  if (status > 0) {
//    if ((fabs(measurement.cte) < 1.0) && (measurement.speed < 1) && settle_down_iterations_++ > 100) {
    if ((fabs(measurement.cte) < 1.0) && (measurement.speed < 1)) {
      measurement.c_dt_secs = 0;
      initialize_fp(ws, measurement);

      last_call_  = system_clock::now();

      // don't call us next time, call the telemetry handler function
      hub_.onMessage(std::bind(&Simulator::OnMessage, this, _1, _2, _3, _4));
    }
  }

  SendManualMode(ws);
}



void Simulator::OnMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  TelemetryMessage measurement;

  auto status = Parse(data, length, &measurement);
  if (0 == status)
    SendManualMode(ws);
  else if (status > 0)
    telemetry_fn(ws, measurement);
}


void Simulator::Run()
{
  static unsigned char msg_shown = 0x0;

  hub_.onMessage(std::bind(&Simulator::SendResetOnMessage, this, _1, _2, _3, _4));

  hub_.onConnection([this](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    if (!(msg_shown & 0x1)) {
      cout << "Connected!!!" << endl;
      msg_shown |= 0x1;
    }
  });

  hub_.onDisconnection([this](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (hub_.listen("127.0.0.1", port)) {
    if (!(msg_shown & 0x2)) {
      cout << "Listening on port " << port << endl;
      msg_shown |= 0x2;
    }
  }
  else {
    cerr << "Failed to listen to port" << endl;
    exit(-1);
  }

  hub_.run();
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

  auto start = line.find_first_of('[');
  auto end = line.find_last_of(']');

  if (start == string::npos || end == string::npos)
    return "";

  return line.substr(start, end - start + 1);
}

void Simulator::SendControl(uWS::WebSocket<uWS::SERVER> &ws, const ControlInput &control) {
  last_control_ = control;
  double throttle = control.throttle;
  double steering = control.steering;

  if (control.throttle > 1)
    throttle = 1;
  else if (throttle < -1)
    throttle = -1;

  if (steering > 1)
    steering = 1;
  else if (steering < -1)
    steering = -1;

  json msgJson = {
      {"steering_angle", steering},
      {"throttle", throttle}
  };

  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//    cout << msg << endl;
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void Simulator::Stop() {
  hub_.getDefaultGroup<true>().close();
}

