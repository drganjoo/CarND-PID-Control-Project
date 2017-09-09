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
  cout << "Initial run: P=" << p[0] << ", " << p[1] << ", " << p[2] << "\t Error: " << best_error_ << endl;

  while (dp[0] + dp[1] + dp[2] > threshold_) {
    for (unsigned int i = 0; i < 3; i++) {
      // increment in the positive direction and check results are inferior or better
      p[i] += dp[i];

      if (!RunGivesBetterResult(i)) {
        // Go in the opposite direction
        p[i] -= 2 * dp[i];

        if (!RunGivesBetterResult(i)) {
          // Both inc / dec give inferior, go back and change dp to a lower value
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
  }
}


void Twiddle::StartCheckBoth() {
  OpenLogFiles();

  best_error_ = Run();
  cout << "Initial run: P=" << p[0] << ", " << p[1] << ", " << p[2] << "\t Error: " << best_error_ << endl;

  while (dp[0] + dp[1] + dp[2] > threshold_) {
    for (unsigned int i = 0; i < 3; i++) {
      // increment in the positive direction and check results are inferior or better
      p[i] += dp[i];

      cout << "P was: " << p[i] - dp[i] << ", trying higher: " << p[i] << endl;
      double inc_error = Run();
      PrintParams(inc_error);
      WriteResultToLog(inc_error);

      // lets try the other direction
      p[i] -= 2 * dp[i];

      cout << "P was: " << p[i] + dp[i] << ", trying lower: " << p[i] << endl;

      double dec_error = Run();
      PrintParams(dec_error);
      WriteResultToLog(dec_error);

      // get the better of the two
      if (inc_error < dec_error) {
        cout << "higher: " << inc_error << " was better than lower: " << dec_error;

        if (inc_error < best_error_) {
          cout << ", and even better than the overall best: " << best_error_ << endl;
          best_error_ = inc_error;
          // go back to the incremented state
          p[i] += 2 * dp[i];
          dp[i] *= 1.1;
        } else {
          cout << ", but not better than the overall best: " << best_error_ << endl;

          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
      else {
        cout << "lower: " << dec_error << " was better than higher: " << inc_error;

        if (dec_error < best_error_) {
          cout << ", and even better than the overall best: " << best_error_ << endl;
          best_error_ = dec_error;
          dp[i] *= 1.1;
        }
        else {
          cout << ", but not better than the overall best: " << best_error_ << endl;

          // go back to the original state since neither incrementing nor decrementing
          // gave us a better result
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
  }
}


bool Twiddle::RunGivesBetterResult(unsigned int i) {
  double error = Run();

  PrintParams(error);
  WriteResultToLog(error);

  bool better = error < best_error_;

  if (better) {
    SaveAndLogBestError(i, error);

    dp[i] *= 1.1;   // next time look for bigger inc / dec.
  }

  return better;
}

void Twiddle::WriteResultToLog(double error){
  if (result_file_.is_open()) {
    result_file_ << p[0] << "," << p[1] << "," << p[2]
                 << "," << dp[0] << "," << dp[1] << "," << dp[2] << error << endl;
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

  cout << "Twidle Ran For\tP = " << p_ss.str() << "\tDP = " << dp_ss.str()
       << "\tBest Error: " << best_error_ << "\trun_error: " << run_error
       << endl;
}


/*----------------------------------------------------------------------------------------*/

CarTwiddle::CarTwiddle(double init_threshold, PIDThrottle *pid_throttle,
                       PIDSteering *pid_steering) : Twiddle(init_threshold),
  pid_throttle_(pid_throttle),
  pid_steering_(pid_steering)
{
}

void CarTwiddle::OnTelemetry(uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
  iterations++;

  ControlInput control;

  control.throttle = pid_throttle_->GetOutput(measurement);
  control.steering = pid_steering_->GetOutput(measurement);

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
    pid_steering_->SetInitialCte(measurement);
    pid_throttle_->SetInitialCte(measurement);

    iterations = 0;
  });

  sim_.OnTelemetry(std::bind(&CarTwiddle::OnTelemetry, this, _1, _2));

  sim_.Run();
  return GetTotalError();
}

/*----------------------------------------------------------------------------------------*/

SteeringTwiddle::SteeringTwiddle(double init_threshold) :
      CarTwiddle(init_threshold, &pid_throttle_, &pid_steering_),
      pid_steering_(p[0], p[1], p[2]),
      pid_throttle_(-0.192793,-0.0125629,-0.3111750)
{
  pid_throttle_.SetDesiredSpeed(20.0);

  p[0] = 0.9;
  dp[0] = 0.04;

  p[1] = 0.09;
  dp[1] = 0.004;

  p[2]  = 0;
  dp[2] = 1;

  calc_after_iterations_ = 800;
  stop_after_iterations_ = 6000;
}

/*----------------------------------------------------------------------------------------*/

ThrottleTwiddle::ThrottleTwiddle(double init_threshold, double desired_speed) :
    CarTwiddle(init_threshold, &pid_throttle_, &pid_steering_),
    pid_steering_(0.1, 0, 0.003),
    pid_throttle_(p[0], p[1], p[2]) {

  p[0] = -0.05;
  dp[0] = 0.04;

  p[1] = -0.005;
  dp[1] = 0.004;

  p[2] = 0;
  dp[2]  = 1;

  calc_after_iterations_ = 60;
  stop_after_iterations_ = 900;

  pid_throttle_.SetDesiredSpeed(desired_speed);
}

void ThrottleTwiddle::OnTelemetry(uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &measurement) {
  static double last_speed = measurement.speed;
  static double speed_integral = 0;

  if (measurement.throttle < 0) {
    double speed_derivative = measurement.speed - last_speed;
    last_speed = measurement.speed;

    if (speed_derivative >= 0) {
      speed_integral += speed_derivative * 0.1;

      // hmmm is the car going in reverse now since the throttle is -ve
      // but speed is increasing. Lets see for the next few seconds to make sure this
      // happens
      if (speed_integral >= 5) {
        // for sure it is going in reverse, lets stop and return a big number from total error

        cout << "stopping!!! car is going in reverse" << endl;
        sim_.Stop();
      }
    }
    else {
      speed_integral = 0;
    }
  } else {
    speed_integral = 0;
  }

  CarTwiddle::OnTelemetry(ws, measurement);
}