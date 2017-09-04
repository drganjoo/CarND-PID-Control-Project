#include "PID.h"
#include <chrono>

using namespace std;
using namespace std::chrono;

PID::PID() {
  p_error = 0;
  i_error = 0;
  d_error = 0;
  Kp = 0.0;
  Ki = 0.0;
  Kd = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double init_cte) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->last_cte_ = init_cte;

  last_time_ = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
}

void PID::UpdateError(double cte) {
  milliseconds now  = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
  double dt = now.count() - last_time_.count();
  last_time_ = now;

  //if (dt == 0)
    dt = 0.1;

  p_error = cte;
  d_error = (last_cte_ - cte) / dt;
  i_error += cte * dt;
  total_error_ += cte * cte;
}

double PID::TotalError() {
  return total_error_;
}

void PID::SetLastCte(double cte) {
  last_cte_ = cte;
}

double PID::GetOutput() {
  const double output = -Kp * p_error - Kd * d_error - Ki * i_error;
  return output;
}