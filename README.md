# KalmanFilter
A very simple and versatile Kalman filter

Kalman filtering is an algorithm for stabilizing measurements that are exposed to noise.
The method uses statistical probability to estimate the "true value" behind the noisy input.
Compared to many other filtering methods (such as low-pass filtering), the Kalman filter does not slow down the response, as it estimates both position and velocity of the value.

This implementation of the Kalman algorithm has a number of advantages:
* No dependencies, libararies or platform related stuff - just pure C++
* Simple to implement, use and understand
* Filters any number of inputs - with only a small impact on memory and processing time. This makes it very suitable for filtering multiple positions - e.g. camera tracking with varying number of interest points.
* Prediction takes time difference (delta time) into account, making it suitable for frame-rate varying systems.

## Example: filtering a single input (Arduino code)
```cpp
#include <kalman.h>

KalmanFilter kf;
unsigned long timer = 0;

void setup() {

  // Set initial state
  kf.set(analogRead(0));
}

void loop() {

  // Delta time : time since last prediction
  float dt = (millis()-timer)/1000.f;
  timer = millis();

  // Kalman filter steps
  kf.predict(dt);
  int x = kf.get();
  kf.correct(analogRead(0));
}

```

## Adjusting parameters
The Kalman algorithm has two parameters that affects the strength of filtering, but also the responsiveness of the output.
* Q - Process noise (covariance). The amount the model is updated on every prediction.
* R - Measurement noise (covariance). The amount the input measurement affects the state of the filter.
In general, the higher these values are, the stronger the filter.

```cpp
kf.setProcessNoise(0.1, 0.01); // Position, velocity
kf.setMeasurementNoise(0.1);
```

## Example: filtering a position (pseudo code)
```cpp
KalmanFilter kf;
float mouse[2];

void setup() {
  kf.init(2);
  kf.setProcessNoise(0.1, 0.01);
  kf.setMeasurementNoise(0.1);
}

void update() {
  float dt = getLastFrameTime();
  kf.predict(dt);
  kf.get(mouse, 2);
  kf.correct(getMousePos(), 2);
}
```
## Classes

* KalmanFilter - The main class. Contains the noise parameters and any number of kalman models
* KalmanModel – The class that models the measurements. Holds the current state (x) and the error covariance (P)
* Vec2 - Vector or 1x2 matrix including various math operations.
* Mat2x2 - Matrix 2x2 including various math operations.
