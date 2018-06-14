#pragma once

#include "ofMain.h"
#include "kalman.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		
        KalmanFilter kf;
};
