#ifndef PID_H
#define PID_H

#include <ostream>
#include <chrono>
#include <string>
#include "Simulator.h"

class PID {
 public:
  PID();
  PID(std::string debug_name);

  virtual ~PID() = default;

  virtual void Init(double Kp, double Ki, double Kd, double init_cte);
  virtual void UpdateError(double cte, double dt_secs, bool include_in_error = false);

  double TotalError();
  double GetOutput();

  double GetDError() {
      return d_error_;
  }

  double GetKp() {
      return kp_;
  }

  double GetKi() {
      return ki_;
  }

 private:
  double p_error_;
  double i_error_;
  double d_error_;
  double kp_;
  double ki_;
  double kd_;
  double total_error_;
  std::string debug_name_;
};

class PIDThrottle : public PID{
 public:
  PIDThrottle();
  PIDThrottle(double desired_speed);
  PIDThrottle(std::string debug_name) : PID(debug_name){}

  ~PIDThrottle() = default;

  void Init(double Kp, double Ki, double Kd, double init_cte) override ;
  virtual void Init(double init_kp, double init_ki, double init_kd, const TelemetryMessage &measurement);

  void UpdateMeasurement(const TelemetryMessage &measurement, bool include_in_error = false);
  void SetDesiredSpeed(const double desired_speed) {
    desired_speed_ = desired_speed;
  }

  inline double GetCte(const TelemetryMessage &measurement) {
    return desired_speed_ - measurement.speed;
  }

 private:
  double desired_speed_;
};

#endif /* PID_H */
