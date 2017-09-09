#ifdef MSVC
#pragma warning(push)
#pragma warning(disable: 4514)
#pragma warning(disable: 4820)
#endif
#include <cassert>
#include "PID.h"
#ifdef MSVC
#pragma warning(pop)
#endif

using namespace std;
using namespace std::chrono;

/*---------------------------------------------------------------------------------*/
PID::PID(double init_kp, double init_ki, double init_kd) :
    kp_(init_kp),
    ki_(init_ki),
    kd_(init_kd)
{
}

void PID::SetInitialCte(const TelemetryMessage &measurement) {
  p_error_ = GetCte(measurement);
}

double PID::GetOutput(const TelemetryMessage &measurement) {
  const double cte = GetCte(measurement);
  double cte_dt = cte * measurement.dt_secs;

  d_error_ = (cte - p_error_) / measurement.dt_secs;
  p_error_ = cte;
  i_error_ += cte_dt;

  accum_error_ += cte_dt * cte_dt;

  double output = kp_ * p_error_ + ki_ * i_error_ + kd_ * d_error_;
  if (output > 1)
    output = 1;
  else if (output < -1)
    output = -1;

  return output;
}
