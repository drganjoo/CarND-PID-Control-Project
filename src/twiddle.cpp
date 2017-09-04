#include <uWS/uWS.h>
#include <iostream>
#include "twiddle.h"
#include "Simulator.h"

using namespace std;

void Twiddle::Start()
{
	double best_error = Run();

	while (dp[0] + dp[1] + dp[2] < threshold_) {
		for (unsigned int i = 0; i < 3; i++) {
			// increase p value in the positive direction and see if we can get a better
			// result with that.
			p[i] += dp[i];

			double error = Run();

			if (error < best_error) {
				error = best_error;
				dp[i] *= 1.1;
			}
			else {
				// Since we did not get a better result in the positive direction
				// lets check P in the other direction (decrease it) to see if we 
				// get a better result
				p[i] -= 2 * dp[i];

				double error = Run();

				if (error < best_error) {
					best_error = error;
					p[i] *= 1.1;
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
}

double Twiddle::Run()
{
	Simulator s;
	s.OnInitialize([&s](uWS::WebSocket<uWS::SERVER> &ws) {
		s.SendReset(ws);
	});

	s.OnTelemetry([&s](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &m) {


	});

	s.Run();

	return 0;
}