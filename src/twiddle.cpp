#include <uWS/uWS.h>
#include "twiddle.h"
#include "Simulator.h"
#include <fstream>

using namespace std;

const int ITERATIONS_TO_RUN = 800;

Twiddle::Twiddle(double init_threshold) {
	threshold_ = init_threshold;

	p[0] = 0.1;
	p[1] = 0.0;
	p[2] = 0.0;
	dp[0] = 0.5;
	dp[1] = 0.5;
	dp[2] = 0.5;
}

void Twiddle::Start()
{
	double best_error = Run();

//    char buffer[512];
//    getcwd(buffer, sizeof(buffer));

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
				best_error = error;
				dp[i] *= 1.1;

                if (best_file.is_open()) {
                    best_file << best_error << "," << p[0] << "," << p[1] << "," << p[2] << endl;
                    best_file.flush();
                }
			}
			else {
				// Since we did not get a better result in the positive direction
				// lets check P in the other direction (decrease it) to see if we 
				// get a better result
				p[i] -= 2 * dp[i];

				error = Run();

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

    if (best_file.is_open())
        best_file.close();

    if (result_file.is_open())
        result_file.close();
}

double ThrottleTwiddle::Run()
{
	static bool first_time = true;

	if (first_time) {
		first_time = false;
		return 	90000;  // difference of 30 * .1 ** 2 = 9 each time called 100 times
	}

	double desired_speed = 30;
	int iterations = 0;

	PID pid_throttle("throttle");
	PID pid_steering("steering");
	Simulator s;

	s.OnInitialize([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &tm) {
		s.SendReset(ws);

		pid_throttle.Init(p[0], p[1], p[2], desired_speed - tm.speed);
		pid_steering.Init(0.1, 0, 0.003, tm.cte);
	});

	s.OnTelemetry([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &m) {
		ControlInput control;

		auto speed_cte = m.speed - desired_speed;
		pid_throttle.UpdateError(speed_cte, iterations++ > 100);
		control.throttle = pid_throttle.GetOutput();

		pid_steering.UpdateError(m.cte);
		control.steering = pid_steering.GetOutput();

		cout << iterations << ": Measurement --> cte:" << m.cte
			<< ", speed: " << m.speed << ", angle: " << m.angle
			<< " Control ->  steering: " << control.steering
			<< " throttle: " << control.throttle
			<< " ** P,I,D is: " << p[0] << ", " << p[1] << ", " << p[2] << endl;

		s.SendControl(ws, control);
		if (iterations > 200) {
			s.Stop();
		}
	});

	s.Run();

	return pid_throttle.TotalError();
}

double SteeringTwiddle::Run()
{
	const double desired_speed = 30;
	int iterations = 0;

	PID pid_throttle("throttle");
	PID pid_steering("steering");
	Simulator s;

	s.OnInitialize([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &tm) {
		s.SendReset(ws);

		pid_steering.Init(p[0], p[1], p[2], tm.cte);
		pid_throttle.Init(1.56807,0.00243957,-0.0972004, desired_speed - tm.speed);
	});

	s.OnTelemetry([&](uWS::WebSocket<uWS::SERVER> &ws, const TelemetryMessage &m) {
		ControlInput control;

		auto speed_cte = m.speed - desired_speed;
		pid_throttle.UpdateError(speed_cte);
		control.throttle = pid_throttle.GetOutput();

		pid_steering.UpdateError(m.cte, iterations++ > ITERATIONS_TO_RUN);
		control.steering = pid_steering.GetOutput();

		cout << iterations << ": Measurement --> cte:" << m.cte
			 << ", speed: " << m.speed << ", angle: " << m.angle
			 << " Control ->  steering: " << control.steering
			 << " throttle: " << control.throttle
			 << " ** P,I,D is: " << p[0] << ", " << p[1] << ", " << p[2] << endl;

		s.SendControl(ws, control);
		if (iterations > ITERATIONS_TO_RUN * 2) {
			s.Stop();
		}
	});

	s.Run();

	return pid_steering.TotalError();
}
