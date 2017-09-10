
[//]: # (Image References)
[pid_1_0_0]: ./videos/pid_1_0_0.mp4
[pid_0.09_0_0]: ./videos/pid_0.09_0_0.mp4
[pid_0.9_0_1000]: ./videos/pid_0.9_0_1000.mp4
[pid_0.9_0_100]: ./videos/pid_0.9_0_100.mp4
[pid_0.9_0_10]: ./videos/pid_0.9_0_10.mp4
[pid_0.9_0_50]: ./videos/pid_0.9_0_10.mp4

## Trial Runs

## Fix P Value:

### A very high value (Kp = -1)

Too much oscillation, unusable:

![pid_1_0_0]

### Good Value with Osciallation (Kp = -0.09)

Oscillates, but usable

![pid_1_0_0]

### PD incorporated, a large value (Kd = -1000)

Very jerky movements of the steering wheel:

![pid_0.9_0_1000]

### PD, smaller value (Kd = -100)

Works very well:

![pid_0.9_0_100]

### PD smaller value (Kd = -10)

Works very well:

![pid_0.9_0_10]

### PD, smaller value (Kd = -50)

Works very good:

![pid_0.9_0_50]
