#ifndef __TWIDDLE__
#define __TWIDDLE__

#include "PID.h"
#include "Simulator.h"

#include <string>

class Twiddle
{
public:
		explicit Twiddle(double init_threshold);
		void Start();

protected:
		virtual double Run() = 0;
		virtual std::string GetFileSuffix() = 0;

protected:
		double p[3];
		double dp[3];
		double threshold_;
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
	explicit CarTwiddle(double init_threshold) : Twiddle(init_threshold),
                                                 pid_throttle_("throttle"),
                                                 pid_steering_("steering")
    {
	}

    virtual void SetTwiddleParams(const TelemetryMessage &measurement) = 0;
	virtual double Run();

protected:
    PID pid_throttle_;
    PID pid_steering_;

    int calc_after_iterations_ = 300;
    int stop_after_iterations_ = 3000;
};

class SteeringTwiddle : public CarTwiddle
{
public:
    explicit SteeringTwiddle(double init_threshold) : CarTwiddle(init_threshold) {
    }

    virtual std::string GetFileSuffix() {
        return "steering";
    }

    virtual void SetTwiddleParams(const TelemetryMessage &measurement);
};


class ThrottleTwiddle : public CarTwiddle
{
public:
    explicit ThrottleTwiddle(double init_threshold, double desired_speed = 30) : CarTwiddle(init_threshold) {
        desired_speed_ = desired_speed;
    }

    virtual std::string GetFileSuffix() {
        return "throttle";
    }

    virtual void SetTwiddleParams(const TelemetryMessage &measurement);

    double desired_speed_;
};

#endif // !__TWIDDLE__

