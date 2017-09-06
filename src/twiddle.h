#ifndef __TWIDDLE__
#define __TWIDDLE__

#include "PID.h"

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

class ThrottleTwiddle : public Twiddle
{
public:
	explicit ThrottleTwiddle(double init_threshold) : Twiddle(init_threshold){
	}

	virtual double Run();
	virtual std::string GetFileSuffix() {
		return "throttle";
	}
};

class SteeringTwiddle: public Twiddle
{
public:
	explicit SteeringTwiddle(double init_threshold) : Twiddle(init_threshold) {
	}

	virtual double Run();
	virtual std::string GetFileSuffix() {
		return "steering";
	}
};

#endif // !__TWIDDLE__

