#ifndef __TWIDDLE__
#define __TWIDDLE__

#include "PID.h"
#include "Simulator.h"
#include "SpeedController.h"
#include <string>
#include <fstream>
#include <memory>
#include <array>
#include <functional>

struct TwiddleParams
{
  double p[3];
  double dp[3];

  TwiddleParams(std::initializer_list<double> init_p, std::initializer_list<double> init_dp) {
    std::copy(init_p.begin(), init_p.end(), p);
    std::copy(init_dp.begin(), init_dp.end(), dp);
  }
};


template <typename Runner>
class Twiddle
{
 public:
  explicit Twiddle(double init_threshold, const TwiddleParams &params) {
    threshold_ = init_threshold;
    std::copy(&params.p[0], &params.p[2], p);
    std::copy(&params.dp[0], &params.dp[2], dp);
    best_error_ = INT_MAX;
  }

  virtual ~Twiddle() {
    if (best_file_.is_open())
      best_file_.close();
    if (result_file_.is_open())
      result_file_.close();
  }

  void Run() {
    RunForIndices({0,1,2});
  }

  void Run(int index) {
    RunForIndices({index});
  }

  void RunForIndices(std::vector<int> indices) {
    OpenLogFiles();

    best_error_ = RunSimulation();
    std::cout << "Initial run: P=" << p[0] << ", " << p[1] << ", " << p[2] << "\t Error: " << best_error_ << std::endl;

    while (DPHigherThanThreshold(indices)) {
      double error;

      for (int index : indices){
        // increment in the positive direction and check results are inferior or better
        p[index] += dp[index];

        std::cout << "Trying higher value for K[" << index << "] = " << p[index];

        if (RunGivesBetterResult(&error)) {
          dp[index] *= 1.1;
        }
        else {
          std::cout << ", which does not gives us a better result. Best = \t" << best_error_ << " < Error: " << error << std::endl;

          // Go in the opposite direction
          p[index] -= 2 * dp[index];

          std::cout << "Trying a lower value for K[" << index << "] = " << p[index];

          if (RunGivesBetterResult(&error)) {
            dp[index] *= 1.1;   // next time look for bigger inc / dec.
          }
          else {
            std::cout << ", which does not gives us a better result. Best = \t" << best_error_ << " < Error: " << error << std::endl;

            // Both inc / dec give inferior, go back and change dp to a lower value
            p[index] += dp[index];
            dp[index] *= 0.9;

            std::cout << "Making P go back to " << p[index] << " BUT changing dp[" << index << "] to " << dp[index] << std::endl;
          }
        }
      }
    }
  }

  bool DPHigherThanThreshold(const std::vector<int> &indices) {
    double sum = 0;
    for (int index: indices) {
      sum += dp[index];
    }

    return sum > threshold_;
  }


 protected:
  double RunSimulation() {
    std::unique_ptr<Runner> runner(new Runner(p[0], p[1], p[2]));
    double error = runner->Run();
    runner.reset();

    WriteResultToLog(error);
    return error;
  }

  bool RunGivesBetterResult(double *error) {
    *error = RunSimulation();

    if (*error < best_error_) {
      best_error_ = *error;
      LogBest();
      return true;
    }

    return false;
  }

  void WriteResultToLog(double error){
    if (result_file_.is_open()) {
      result_file_ << p[0] << "," << p[1] << "," << p[2]
                   << "," << dp[0] << "," << dp[1] << "," << dp[2] << error << std::endl;
    }
  }

  void OpenLogFiles() {
    best_file_.open("./" + Runner::GetFilePrefix() + "_best.log");
    if (!best_file_.is_open()) {
      std::cout << "Could not open best_throttle.log file. Error:" << strerror(errno) << std::endl;
    }

    result_file_.open("./" + Runner::GetFilePrefix()  + "_result.log");
    if (!result_file_.is_open()) {
      std::cout << "Could not open best_throttle.log file. Error:" << strerror(errno) << std::endl;
    }
  }

  void LogBest() {
    if (best_file_.is_open()) {
      best_file_ << p[0] << "," << p[1] << "," << p[2] << best_error_ << std::endl;
      best_file_.flush();
    }
  }

 protected:
  double p[3];
  double dp[3];
  double threshold_;
  double best_error_;
  std::ofstream best_file_;
  std::ofstream result_file_;
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

class SteeringTwiddle
{
 public:
  static std::string GetFilePrefix() {
    return "steering";
  }

  SteeringTwiddle(double kp, double ki, double kd);

  double Run();

 protected:
  const unsigned int calc_after_iterations_ = 600;
  const unsigned int stop_after_iterations_ = 6000;

  unsigned int iterations_ = 0;
  PIDSteering pid_steering_;
  SpeedController speed_controller_;
  Simulator sim_;
};

/*----------------------------------------------------------------------------------------*/

class ThrottleTwiddle
{
 public:
  static std::string GetFilePrefix() {
    return "throttle";
  }

  ThrottleTwiddle(double kp, double ki, double kd);

  double Run();

 private:
  bool IsCarGoingInReverse(const TelemetryMessage &measurement);

 protected:
  const unsigned int calc_after_iterations_ = 60;
  const unsigned int stop_after_iterations_ = 900;

  unsigned int iterations_ = 0;
  PIDSteering pid_steering_;
  PIDThrottle pid_throttle_;
  Simulator sim_;
};

#endif // !__TWIDDLE__

