#include <uWS/uWS.h>
#include "twiddle.h"
#include "Simulator.h"
#include <fstream>
#include <functional>

using namespace std;
using namespace std::placeholders;


Twiddle::Twiddle(double init_threshold) {
  threshold_ = init_threshold;

  p[0] = 0.0;
  p[1] = 0.0;
  p[2] = 0.0;
  dp[0] = 1;
  dp[1] = 1;
  dp[2] = 1;
}

void Twiddle::Start()
{
  double best_error = Run();

  ofstream best_file, result_file;
  best_file.open("./" + GetFileSuffix() + "_best.log");
  if (!best_file.is_open()) {
    cout << "Could not open best_throttle.log file. Error:" << std::strerror(errno) << endl;
  }

  result_file.open("./" + GetFileSuffix() + "_result.log");
  if (!result_file.is_open()) {
    cout << "Could not open best_throttle.log file. Error:" << std::strerror(errno) << endl;
  }

  while (dp[0] + dp[1] + dp[2] > threshold_) {
    for (unsigned int i = 0; i < 3; i++) {
      // increase p value in the positive direction and see if we can get a better
      // result with that.
      p[i] += dp[i];

      cout << "Using p[0]: " << p[0] << endl;

      double error = Run();

      if (result_file.is_open()) {
        result_file << error << "," << p[0] << "," << p[1] << "," << p[2]
                    << "," << dp[0] << "," << dp[1] << "," << dp[2] << endl;
        result_file.flush();
      }

      if (error < best_error) {
        SetBestError(i, error, &best_error, best_file);
      }
      else {
        // Since we did not get a better result in the positive direction
        // lets check P in the other direction (decrease it) to see if we
        // get a better result
        p[i] -= 2 * dp[i];

        error = Run();

        if (error < best_error) {
          SetBestError(i, error, &best_error, best_file);
        }
        else {
          // since neither increasing nor decreasing value of P in either direction
          // gives us a better result, lets go back to the original P and decrease
          // the dp value so that next time we will try some shorter range of +ve/-ve
          // changes in P
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
  }

  if (best_file.is_open())
    best_file.close();

  if (result_file.is_open())
    result_file.close();
}
void Twiddle::SetBestError(unsigned int i, double error, double *best_error, ofstream &best_file) {
  *best_error = error;
  dp[i] *= 1.1;

  if (best_file.is_open()) {
    best_file << best_error << "," << p[0] << "," << p[1] << "," << p[2] << endl;
    best_file.flush();
  }

  cout << "Best Error: " << best_error << " ** P,I,D is: " << p[0] << ", " << p[1] << ", " << p[2] << endl;
}

CarTwiddle::CarTwiddle(double init_threshold) : Twiddle(init_threshold),
                                                pid_throttle_("throttle"),
                                                pid_steering_("steering")
{
}

void CarTwiddle::OnTelemetry(uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
  static unsigned int iterations = 0;
  ControlInput control;

  iterations++;

  auto speed_cte = GetSpeedCte(measurement);
  pid_throttle_.UpdateError(speed_cte, iterations > calc_after_iterations_);
  control.throttle = pid_throttle_.GetOutput();

  pid_steering_.UpdateError(measurement.cte, iterations > calc_after_iterations_);
  control.steering = pid_steering_.GetOutput();

  //		cout << iterations << ": Measurement --> cte:" << measurement.cte
  //			 << ", speed: " << measurement.speed << ", angle: " << measurement.angle
  //			 << " Control ->  steering: " << control.steering
  //			 << " throttle: " << control.throttle
  //			 << " ** P,I,D is: " << p[0] << ", " << p[1] << ", " << p[2] << endl;

  sim_.SendControl(ws, control);

  if (iterations > stop_after_iterations_) {
    sim_.Stop();
  }
}

double CarTwiddle::Run() {
  sim_.OnInitialize([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
    SetTwiddleParams(measurement);
  });

  sim_.OnTelemetry(std::bind(&CarTwiddle::OnTelemetry, this, _1, _2));

  sim_.Run();

  return GetTotalError();
}

void SteeringTwiddle::SetTwiddleParams(const TelemetryMessage &measurement) {
  pid_steering_.Init(p[0], p[1], p[2], measurement.cte);
  pid_throttle_.Init(1.56807,0.00243957,-0.0972004, 30 - measurement.speed);
}

SteeringTwiddle::SteeringTwiddle(double init_threshold) : CarTwiddle(init_threshold) {
  p[0] = 0.2;
  dp[0] = 0.1;
}

void ThrottleTwiddle::SetTwiddleParams(const TelemetryMessage &measurement) {
  pid_steering_.Init(0.1, 0, 0.003, measurement.cte);
  pid_throttle_.Init(p[0], p[1], p[2], GetSpeedCte(measurement));
}

ThrottleTwiddle::ThrottleTwiddle(double init_threshold, double desired_speed /*= 30 */) : CarTwiddle(init_threshold) {
  desired_speed_ = desired_speed;

  p[0] = -0.1;
  p[1] = -0.1;
  dp[0] = 0.1;
  dp[1] = 0.1;

  calc_after_iterations_ = 10;
  stop_after_iterations_ = 900;
}

double ThrottleTwiddle::Run() {
  if (pid_throttle_.GetKp() > 0 || pid_throttle_.GetKi() > 0)
    return INT_MAX;

  return CarTwiddle::Run();
}
