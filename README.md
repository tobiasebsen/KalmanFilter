# KalmanFilter
A very simple and versatile Kalman filter

This implementation of the Kalman algorithm has a number of advantages:
* No dependencies, libararies or platform related stuff - just pure C++
* Simple to implement, use and understand
* Filters any number of inputs - with only a small impact on memory and processing time. This makes it very suitable for filtering multiple positions - e.g. blob tracking with varying number of blobs.

## Example: filtering a single input
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

## Example: filtering a position
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
