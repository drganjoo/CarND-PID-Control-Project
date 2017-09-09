#ifndef __TWIDDLE__
#define __TWIDDLE__

#include "PID.h"
#include "Simulator.h"
#include <string>
#include <fstream>
#include <memory>

class Twiddle
{
 public:
  explicit Twiddle(double init_threshold);
  virtual ~Twiddle();

  void Start();
  void StartCheckBoth();

 protected:
  void SaveAndLogBestError(unsigned int i, double error);
  virtual double Run() = 0;
  virtual std::string GetFileSuffix() = 0;
  void PrintParams(double run_error);

 protected:
  double p[3];
  double dp[3];
  double threshold_;
  double best_error_;
  std::ofstream best_file_;
  std::ofstream result_file_;

  void OpenLogFiles();
  void WriteResultToLog(double error);
  bool RunGivesBetterResult(unsigned int i);
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

/*----------------------------------------------------------------------------------------*/

class CarTwiddle: public Twiddle
{
 public:
  explicit CarTwiddle(double init_threshold, PIDThrottle *pid_throttle, PIDSteering *pid_steering);
  virtual ~CarTwiddle() = default;

  virtual double GetTotalError() = 0;
  virtual double Run() override;

 protected:
  virtual void OnTelemetry(uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement);

 protected:

  Simulator sim_;
  PIDThrottle *pid_throttle_;
  PIDSteering *pid_steering_;

  unsigned int calc_after_iterations_ = 300;
  unsigned int stop_after_iterations_ = 3000;
  unsigned int iterations = 0;
};

/*----------------------------------------------------------------------------------------*/


class SteeringTwiddle : public CarTwiddle
{
 public:
  explicit SteeringTwiddle(double init_threshold);

  std::string GetFileSuffix() override{
    return "steering";
  }

  double GetTotalError() override{
    return pid_steering_.GetAccumError();
  }

 protected:
  PIDSteering pid_steering_;
  PIDThrottle pid_throttle_;
};

/*----------------------------------------------------------------------------------------*/

class ThrottleTwiddle : public CarTwiddle
{
 public:
  explicit ThrottleTwiddle(double init_threshold, double desired_speed);

  std::string GetFileSuffix() override{
    return "throttle";
  }

  double GetTotalError() override{
    return pid_throttle_.GetAccumError();
  }

  void OnTelemetry(uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) override;

 protected:
  PIDSteering pid_steering_;
  PIDThrottle pid_throttle_;
};

#endif // !__TWIDDLE__

