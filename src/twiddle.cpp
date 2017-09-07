#include <uWS/uWS.h>
#include "twiddle.h"
#include "Simulator.h"
#include <fstream>
#include <functional>
#include <sstream>
#include <iterator>

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
  best_error_ = INT_MAX;
}

Twiddle::~Twiddle() {
  if (best_file_.is_open())
    best_file_.close();

  if (result_file_.is_open())
    result_file_.close();
}

void Twiddle::Start() {
  OpenLogFiles();

  best_error_ = Run();
  PrintParams(best_error_);

  while (dp[0] + dp[1] + dp[2] > threshold_) {
    for (unsigned int i = 0; i < 3; i++) {
      // increment in the positive direction and check results are inferior or better
      p[i] += dp[i];

      if (RunGivesInferiorResult(i)) {
        // Go in the opposite direction
        p[i] -= 2 * dp[i];

        if (RunGivesInferiorResult(i)) {
          // Both inc / dec give inferior, go back and change dp to a lower value
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
  }
}

bool Twiddle::RunGivesInferiorResult(unsigned int i) {
  double error = Run();
  PrintParams(error);
  WriteResultToLog(error);

  bool inferior = true;
  if (error < best_error_) {
    inferior = false;
    SaveAndLogBestError(i, error);

    // check for a bigger inc / dec next time
    dp[i] *= 1.1;
  }

  return inferior;
}

void Twiddle::WriteResultToLog(double error){
  if (result_file_.is_open()) {
    result_file_ << error << "," << p[0] << "," << p[1] << "," << p[2]
                 << "," << dp[0] << "," << dp[1] << "," << dp[2] << endl;
  }
}

void Twiddle::OpenLogFiles() {
  best_file_.open("./" + GetFileSuffix() + "_best.log");
  if (!best_file_.is_open()) {
    cout << "Could not open best_throttle.log file. Error:" << strerror(errno) << endl;
  }

  result_file_.open("./" + GetFileSuffix() + "_result.log");
  if (!result_file_.is_open()) {
    cout << "Could not open best_throttle.log file. Error:" << strerror(errno) << endl;
  }
}

void Twiddle::SaveAndLogBestError(unsigned int i, double error) {
  best_error_ = error;

  if (best_file_.is_open()) {
    best_file_ << p[0] << "," << p[1] << "," << p[2] << best_error_ << endl;
    best_file_.flush();
  }
}

void Twiddle::PrintParams(double run_error) {
  ostringstream p_ss, dp_ss;
  copy(p, p + 3, ostream_iterator<double>(p_ss, ","));
  copy(dp, dp + 3, ostream_iterator<double>(dp_ss, ","));

  cout << "Twidle Run For\tP = " << p_ss.str() << "\tDP = " << dp_ss.str()
       << "\tBest Error: " << best_error_ << "\trun_error: " << run_error
       << endl;
}


/*----------------------------------------------------------------------------------------*/

CarTwiddle::CarTwiddle(double init_threshold) : Twiddle(init_threshold),
                                                pid_throttle_("throttle"),
                                                pid_steering_("steering")
{
}

void CarTwiddle::OnTelemetry(uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
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
    iterations = 0;
  });

  sim_.OnTelemetry(std::bind(&CarTwiddle::OnTelemetry, this, _1, _2));

  sim_.Run();

  return GetTotalError();
}

/*----------------------------------------------------------------------------------------*/

void SteeringTwiddle::SetTwiddleParams(const TelemetryMessage &measurement) {
  pid_steering_.Init(p[0], p[1], p[2], measurement.cte);
  pid_throttle_.Init(1.56807,0.00243957,-0.0972004, 30 - measurement.speed);
}

SteeringTwiddle::SteeringTwiddle(double init_threshold) : CarTwiddle(init_threshold) {
  p[0] = 0.2;
  dp[0] = 0.1;
}

/*----------------------------------------------------------------------------------------*/

void ThrottleTwiddle::SetTwiddleParams(const TelemetryMessage &measurement) {
  pid_steering_.Init(0.1, 0, 0.003, measurement.cte);
  pid_throttle_.Init(p[0], p[1], p[2], GetSpeedCte(measurement));
}

ThrottleTwiddle::ThrottleTwiddle(double init_threshold, double desired_speed /*= 30 */) : CarTwiddle(init_threshold) {
  desired_speed_ = desired_speed;

  p[0] = -0.05;
  dp[0] = 0.01;

  p[1] = 0.01;
  dp[1] = 0.001;

  p[2] = 0;
  dp[2]  = 1;

  calc_after_iterations_ = 60;
  stop_after_iterations_ = 900;
}

double ThrottleTwiddle::Run() {
  return CarTwiddle::Run();
}

void ThrottleTwiddle::OnTelemetry(uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
  static double last_speed = measurement.speed;
  static unsigned int count_reverse = 0;

  if (measurement.throttle < 0) {
    double d_speed = measurement.speed - last_speed;
    last_speed = measurement.speed;

    if (d_speed >= 0) {
      // hmmm is the car going in reverse now since the throttle is -ve
      // but speed is increasing. Lets see for the next few seconds to make sure this
      // happens
      count_reverse++;

      if (count_reverse >= 30) {
        // for sure it is going in reverse, lets stop and return a big number from total error
        pid_throttle_.UpdateError(INT_MAX, true);
        sim_.Stop();
      }
    }
    else {
      count_reverse = 0;
    }
  } else {
    count_reverse = 0;
  }

  CarTwiddle::OnTelemetry(ws, measurement);
}
