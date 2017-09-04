#ifndef __TWIDDLE__
#define __TWIDDLE__

#include "PID.h"

class Twiddle 
{
public:
	Twiddle(double init_threshold) {
		threshold_ = init_threshold;
	}

	void Start();
	double Run();

protected:
	double p[3];
	double dp[3];
	double threshold_;

};
#endif // !__TWIDDLE__
