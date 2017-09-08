#ifndef __TWIDDLE__
#define __TWIDDLE__

#include "PID.h"
#include "Simulator.h"
#include <string>
#include <fstream>

class Twiddle
{
 public:
  explicit Twiddle(double init_threshold);
  virtual ~Twiddle() = default;

  void Start();

 protected:
  void SetBestError(unsigned int i, double error, std::ofstream &best_file);
  virtual double Run() = 0;
  virtual std::string GetFileSuffix() = 0;
  void PrintParams(double run_error);

 protected:
  double p[3];
  double dp[3];
  double threshold_;
  double best_error_;
  std::ofstream best_file;
  std::ofstream result_file;
};

//class SteeringThrottleTwiddle : public Twiddle
//{
//public:
//	explicit ThrottleTwiddle(double init_threshold) : Twiddle(init_threshold){
//	}
//
//	virtual double Run();
//	virtual std::string GetFileSuffix() {
//		return "throttle";
//	}
//};

class CarTwiddle: public Twiddle
{
 public:
  explicit CarTwiddle(double init_threshold);
  virtual ~CarTwiddle() = default;

  virtual void SetTwiddleParams(const TelemetryMessage &measurement) = 0;
  virtual double GetTotalError() = 0;
  virtual double Run() override;

 protected:
  double GetSpeedCte(const TelemetryMessage &measurement) {
    return desired_speed_ - measurement.speed;
  }

  virtual void OnTelemetry(uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement);

 protected:

  Simulator sim_;
  PID pid_throttle_;
  PID pid_steering_;

  double desired_speed_ = 30;
  int calc_after_iterations_ = 300;
  int stop_after_iterations_ = 3000;
  unsigned int iterations = 0;
};

class SteeringTwiddle : public CarTwiddle
{
 public:
  explicit SteeringTwiddle(double init_threshold);

  virtual std::string GetFileSuffix() override{
    return "steering";
  }

  virtual double GetTotalError() override{
    return pid_steering_.TotalError();
  }

  virtual void SetTwiddleParams(const TelemetryMessage &measurement) override;
};


class ThrottleTwiddle : public CarTwiddle
{
 public:
  explicit ThrottleTwiddle(double init_threshold, double desired_speed = 30);
  virtual std::string GetFileSuffix() override{
    return "throttle";
  }

  virtual double GetTotalError() override{
    return pid_throttle_.TotalError();
  }

  virtual void OnTelemetry(uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) override;
  virtual double Run() override;
  virtual void SetTwiddleParams(const TelemetryMessage &measurement) override;
};

#endif // !__TWIDDLE__

