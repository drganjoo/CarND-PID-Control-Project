#pragma warning(push)
#pragma warning(disable: 4514)
#pragma warning(disable: 4820)
#include "PID.h"
#include <chrono>
#pragma warning(pop)

using namespace std;
using namespace std::chrono;

/*---------------------------------------------------------------------------------*/
PID::PID() {
  p_error = 0;
  i_error = 0;
  d_error = 0;
  Kp = 0.0;
  Ki = 0.0;
  Kd = 0.0;
}

PID::~PID() {}

void PID::Init(double init_kp, double init_ki, double init_kd, double init_cte) {
  this->Kp = init_kp;
  this->Ki = init_ki;
  this->Kd = init_kd;
  this->last_cte_ = init_cte;

  last_time_ = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
}

void PID::UpdateError(double cte) {
  milliseconds now  = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
  double dt = (now.count() - last_time_.count()) / 1000.0;
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
  double output = -Kp * p_error - Kd * d_error - Ki * i_error;
  
  if (output > 1)
	  output = 1;
  else if (output < -1)
	  output = -1;

  return output;
}