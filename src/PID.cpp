#ifdef MSVC
#pragma warning(push)
#pragma warning(disable: 4514)
#pragma warning(disable: 4820)
#endif
#include "PID.h"
#ifdef MSVC
#pragma warning(pop)
#endif

using namespace std;
using namespace std::chrono;

/*---------------------------------------------------------------------------------*/
PID::PID() : PID("") {

}

PID::PID(string debug_name) {
  Init(0, 0, 0, 0);
  debug_name_ = debug_name;
}

void PID::Init(double init_kp, double init_ki, double init_kd, double init_cte) {
  kp_ = init_kp;
  ki_ = init_ki;
  kd_ = init_kd;
  p_error_ = init_cte;
  i_error_ = 0;
  d_error_ = 0;
  total_error_ = 0;
}

void PID::UpdateError(double cte, double dt_secs, bool include_in_error /*= false */) {
  d_error_ = (cte - p_error_) / dt_secs;
  p_error_ = cte;

  double cte_dt = cte * dt_secs;
  i_error_ += cte_dt;

  if (include_in_error)
    total_error_ += cte;
}

double PID::TotalError() {
  return total_error_;
}


double PID::GetOutput() {
  double output = -kp_ * p_error_ - ki_ * i_error_ - kd_ * d_error_ ;

  if (output > 1)
    output = 1;
  else if (output < -1)
    output = -1;

  return output;
}

/*--------------------------------------------------------------------------------------------*/
PIDThrottle::PIDThrottle() : PIDThrottle(30) {

}

PIDThrottle::PIDThrottle(double desired_speed) : PID() {
  desired_speed_ = desired_speed;
}


void PIDThrottle::UpdateError(double cte, double dt_secs, bool include_in_error /*= false */){
  PID::UpdateError(cte, dt_secs, include_in_error);
}

void PIDThrottle::UpdateError(const TelemetryMessage &measurement, bool include_in_error) {
  PID::UpdateError(GetCte(measurement), measurement.dt_secs, include_in_error);
}

void PIDThrottle::Init(double init_kp, double init_ki, double init_kd, const TelemetryMessage &measurement) {
  Init(init_kp, init_ki, init_kd, GetCte(measurement));
}

void PIDThrottle::Init(double Kp, double Ki, double Kd, double init_cte) {
  PID::Init(Kp, Ki, Kd, init_cte);
}
