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
