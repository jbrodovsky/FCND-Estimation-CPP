# Estimation Project

Following the instructions in the read me I set up the project locally and successfully compiled and ran the simulator. I then completed the following tasks.

## Task 1: Sensor Noise

To complete this task I processed the data from the scenario `06_NoisySensors` saved in `config/log/Graph1.txt` and `config/log/Graph2.txt` using Excel. Using the `STDEV.P` command I calculated the standard deviation of the GPS x and y positions to be 0.6978 meters and the IMU x and y accelerometers to be 0.4960 m/s^2. I correspondingly set `MeasuredStdDev_GPSPosXY` and `MeasuredStdDev_AccelXY` in the confiugration file for scenario 6 (`06_SensorNoise.txt`) to 0.6978 and 0.4960 respectfively.

## Task 2: Attitude Estimation

To complete this task I followed the method described in section 7.2 of Estimation for Quadrotors. 

I used equation 43 to define the predicted quaternion:

$\bar{q}_t = dq q_t$

In code I used the `Quaternion` class:

```
Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
attitude.IntegrateBodyRate(V3D(gyro.x, gyro.y, gyro.z), dtIMU);
```

I then set the predicted roll, pitch, and yaw for the nonlinear complimentary filter as the roll, pitch, and yaw developed from the quaternion, wrapping the yaw angle to $2 \pi$.

## Task 3: Prediction Step

In task three I implemented the basic integration step need for the prediction function:

```
V3F acceleration = attitude.Rotate_BtoI(accel);
// Integrate positions
predictedState(0) = curState(0) + curState(3) * dt;
predictedState(1) = curState(1) + curState(4) * dt;
predictedState(2) = curState(2) + curState(5) * dt;
// Integrate velocities
predictedState(3) = curState(3) + acceleration.x * dt;
predictedState(4) = curState(4) + acceleration.y * dt;
predictedState(5) = curState(5) + (acceleration.z - CONST_GRAVITY) * dt;
```
I then tuned parameters `QPosXYStd`, `QPosZStd`, `QVelXYStd`, and `QVelZStd` until I was able to keep the parameters in scenario 9 within the covariance envelope.

## Task 4: Magnetometer Update

I implemented magnetic yaw estimation by completing the `UpdateFromMag` function. `hPrime` was set to all zeros save for the final element which was 1. Additionally, I incorporated the magnetic yaw measurement by correcting it based on the state estimate value. If the difference was over $\pi$ the measurement value was wrapped to $\pm 2\pi$.

```
hPrime(0, 6) = 1;
zFromX(0) = ekfState(6);
float difference = z(0) - zFromX(0);

if (difference < -F_PI) { z(0) += 2.f * F_PI; }
if (difference > F_PI) { z(0) -= 2.f * F_PI; }
```

## Task 5: GPS Update

This was a relatively straight forward implementation. The diagonal elements of $h'(t)$ were set to one and the predicted measurement from the state was simply the first six states.

```
zFromX.setZero();
for (int i = 0; i < 6; i++) 
{
    zFromX(i) = ekfState(i);
    hPrime(i, i) = 1;
}
```

Following the instructions in step 5, I ran this a few times to adjust tuning. First was using the ideal estimator and IMU, which had very good results. This continued to hae good results when switching to a non-ideal estimator. Adding in noisy IMU values decreased performance and required several runs to retune before settling on the final values:

```
# Process noise model
QPosXYStd = .05
QPosZStd = .05
QVelXYStd = .25
QVelZStd = .1
QYawStd = .1

# GPS measurement std deviations
GPSPosXYStd = 5
GPSPosZStd = 7
GPSVelXYStd = .1
GPSVelZStd = .1
```

## Task 6: Adding Your Controller

I then copied my `QuadController.cpp` and `QuadControlParams.txt` from the prior project into this repo. Continuing to run scenario 11, I retuned the controller parameters and achieved passing results.

![]('scenario11.png')