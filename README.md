
[//]: # (Image References)
[pid_1_0_0]: ./videos/pid_1_0_0.mp4
[pid_0.09_0_0]: ./videos/pid_0.09_0_0.mp4
[pid_0.9_0_1000]: ./videos/pid_0.9_0_1000.mp4
[pid_0.9_0_10]: ./videos/pid_0.9_0_10.mp4
[pid_0.09_0_0.1]: ./videos/pid_0.09_0_0.1.mp4
[pid_0.15_0.09_0.2]: ./videos/pid_0.15_0.09_0.2.mp4
[pid_0.195_0.0079_0.2]: ./videos/pid_0.195_0.0079_0.2.mp4

# PID

## Effect of P, I and D on the project

**steering_angle** = kP * CTE + kI * CTE * dt + Kd * (CTE_K - CTE_K-1) / dt

**Propotional Gain**

In order to steer the car back on track, all three parameters had to be in the opposite direction to the cross track error. Propotional gain had the biggest impact on steering. The P value determined how quickly the car responded to the cross track error. A large P value resulted in the car quickly trying to get back but this caused oscillation as it overshot the desired angle.

This video file shows the car using only the P term: ![pid_1_0_0]

**Derivative Gain**
 
In order to stop the oscillation, the steering angle had to be reduced as the cross track error was being reduced. The derivative term helped in this by taking into consideration the change in CTE from one time step to next. The net result of the D term was almost a smooth steering angle

**Integral Gain**

Just using the P and the D component, helped steer the car almost through out the course. However on turnings, where the road was in a curve, just the P and D components alone wasn't proving to be enough to bring the car back onto the center. The I term helped in keeping track of the sum of the errors and consequently increasing in the steer angle.

## Final parameters chosen:

P = -0.195  
I = -0.0079  
D = -0.2

## Tuning

### Twiddle Algorithm

I had originally thought that the twiddle algorithm would turn out to be a **silver bullet** that will automatically find all PID parameters. Therefore I wrote a lot of code to encapsulate the simulator's working to help me implement the twiddle algorithm. (Simulator.cpp/h)

However, I was absolutely shocked, after running it overnight and for days, I realized that it had local optima issues. One time it even declared reverse throttle as the best PID for throttle. Had to figure out a way to detect and declare a high error when the car was going in reverse (PIDThrottle class written in Twiddle.cpp).

Eventually, I realized that Twiddle can only work once some working parameters have been found and then too within some limits of dp. In the end, I have written seperate functions that allow Twiddle to be run on either all three P, I and D OR on just one of them (Twiddle.h - RunForIndices function).

### Final Submission

Had to resort to finding some parameters to work with before twiddling, therefore at the time of this submission I have used the manual tuning following advice from:

[Stackoverflow - Tuning PID Parameters](https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops)

Summary:

1) Come up with some P value that does ok, but car oscillates.  
2) Figure out a D value that can stop the oscillation. Start really big like -1000, see what happens and then bring D down
3) Once P and D are somewhat ok, figure out I by starting off with a very small value
4) In case the car's response is slow, increase P and D accordingly.

Sequence of manual training values for steering:

| P | I | D | Video Link|
|---|---|---|---|
| -1 | 0 | 0 | ![pid_1_0_0]
| -0.5 | 0 | 0 | 
| -0.1 | 0 | 0 | 
| -0.09 | 0 | 0 | ![pid_0.09_0_0]
| -0.09 | 0 | -1000 | ![pid_0.9_0_1000]
| -0.09 | 0 | -100 | 
| -0.09 | 0 | -10 | ![pid_0.9_0_10] 
|-0.09| 0| -0.1| ![pid_0.09_0_0.1]
|-0.15| -0.09| -0.2| ![pid_0.15_0.09_0.2]
| **-0.195** | **-0.0079** | **-0.2** | ![pid_0.195_0.0079_0.2]

## Shortcomings

1) There are a lot of jerky steering movements
2) On the bridge the car steers sharply for a short while and then comes back. Need to figure out if CTE is bad at that stage or something is wrong with the algorithm.
3) Had to reset the I_Error, otherwise the car was taking much longer to take the right turn towards the end

## Lessons Learned

1) Twiddle won't act like magic and it will not figure out perfect parameters, specially if the starting values are kept like in the lesson, P = (0,0,0), DP = (1,1,1)
2) Unless one knows how to do some sort of PID tuning manually, it is very difficult to understand any algorithmic method of tuning.
3) Stick to slower but consistent speed and make the PID for steering work. I spent a lot of time doing speed control, thinking that it was the high speed that wasn't letting twiddle work.
4) After reseting the simulator, for a few iterations, old CTE values are being sent. One has to wait a few iterations for that to get stable.

## Code Explanation

**Simulator Class**

Encapsulates the communication with the simulator. Two functions are provided for callbacks:

. OnInitialize (Called only the first time simulator connects)  
. OnTelemetry (each time a data packet of **telemetry** is sent) 

Values sent by the simulator are encapsulated in a structure:

```
struct TelemetryMessage {
  double cte;
  double speed;
  double angle;
  double c_throttle;
  double c_dt_secs;
};
```

Along with the sensor readings, two additional calculated values are also sent to the reset of the program. *c_throttle* holds the last throttle command (used only for debugging) and *c_dt_secs* is the **delta time** passed since the last reading.

*Note: on my computers, dt_secs comes out to be really fast like 6ms on average*

PID.cpp/PID.h

On Initialization, p_error is set to the initial CTE sent by the simulator:

```
void PID::SetInitialCte(const TelemetryMessage &measurement) {
  p_error_ = GetCte(measurement);
}
```

```
double PID::GetOutput(const TelemetryMessage &measurement) {
  const double cte = GetCte(measurement);
  double cte_dt = cte * measurement.c_dt_secs;

  d_error_ = (cte - p_error_) / measurement.c_dt_secs;
  p_error_ = cte;
  i_error_ += cte_dt;

  accum_error_ += cte_dt * cte_dt;

  double output = kp_ * p_error_ + ki_ * i_error_ + kd_ * d_error_;
  if (output > 1)
    output = 1;
  else if (output < -1)
    output = -1;

  return output;
}

```

dt_secs (dt in secs) is calculated by the Simulator class:

```
auto now = system_clock::now();
measurement->c_dt_secs = duration_cast<milliseconds>(now - last_call_).count();
measurement->c_dt_secs /= 1000.0;
```

Twiddle Algorithm Implementation:

Since there are two different PID's in the program that needed to be tuned, a base class Twiddle has been written that takes a template parameter for either SteeringTwiddle or ThrottleTwiddle. Difference between the two being that Steering uses a nominal working Throttle PID to have a consistent speed. Throttle on the other hand requires a nominal working PID for steering to get some room to twiddle throttle values.

Both work for different iterations before declaring their errors. Also, the ThrottleTwiddle has code to detect car going in reverse and penalizes the particular P's in that case.

```
  void RunForIndices(std::vector<int> indices) {
    OpenLogFiles();

    best_error_ = RunSimulation();

    while (DPHigherThanThreshold(indices)) {
      double error;

      for (int index : indices){
        // increment in the positive direction and check results are inferior or better
        p[index] += dp[index];

        if (RunGivesBetterResult(&error)) {
          dp[index] *= 1.1;
        }
        else {
          // Go in the opposite direction
          p[index] -= 2 * dp[index];

          if (RunGivesBetterResult(&error)) {
            dp[index] *= 1.1;   // next time look for bigger inc / dec.
          }
          else {
            // Both inc / dec give inferior, go back and change dp to a lower value
            p[index] += dp[index];
            dp[index] *= 0.9;
          }
        }
      }
    }
  }
```

```
  bool RunGivesBetterResult(double *error) {
    *error = RunSimulation();

    if (*error < best_error_) {
      best_error_ = *error;
      LogBest();
      return true;
    }

    return false;
  }
```