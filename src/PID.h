#ifndef PID_H
#define PID_H

#include <ostream>
#include <chrono>
#include <string>
#include "Simulator.h"

class PID {
 public:
  PID(double init_kp, double init_ki, double init_kd);

  virtual ~PID() = default;

  void SetInitialCte(const TelemetryMessage &measurement);
  double GetOutput(const TelemetryMessage &measurement);

  inline double GetAccumError() const{ return accum_error_; }

  inline double GetPError() const{ return p_error_;}
  inline double GetIError() const{ return i_error_;}
  inline double GetDError() const{ return d_error_;}

  inline double GetKp() const{ return kp_; }
  inline double GetKi() const{ return ki_; }
  inline double GetKd() const{ return kd_; }

  inline void ResetTotalError() { accum_error_ = 0; }

 protected:
  virtual double GetCte(const TelemetryMessage &measurement) = 0;

 protected:
  const double kp_;
  const double ki_;
  const double kd_;
  double p_error_;
  double i_error_;
  double d_error_;
  double accum_error_;
  std::string debug_name_;
};


class PIDSteering : public PID {
 public:
  PIDSteering(double init_kp, double init_ki, double init_kd) :
      PID(init_kp, init_ki, init_kd){
  }

  double GetCte(const TelemetryMessage &measurement) override {
    return measurement.cte;
  }
};


class PIDThrottle : public PID{
 public:
  PIDThrottle(double init_kp, double init_ki, double init_kd) :
      PID(init_kp, init_ki, init_kd)
  {
    desired_speed_ = 30;
  }

  ~PIDThrottle() = default;

  void SetDesiredSpeed(const double desired_speed) {
    desired_speed_ = desired_speed;
  }

  double GetCte(const TelemetryMessage &measurement) override {
    return desired_speed_ - measurement.speed;
  }

 private:
  double desired_speed_;
};

#endif /* PID_H */
