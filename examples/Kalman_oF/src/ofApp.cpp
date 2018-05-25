#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

    kf.init(2);
}

//--------------------------------------------------------------
void ofApp::update(){

    ofVec2f z(ofGetMouseX(), ofGetMouseY());
    z += ofRandom(-10, 10);
    
    float dt = ofGetLastFrameTime();
    kf.predict(dt);
    kf.correct(z.getPtr(), 2);
}

//--------------------------------------------------------------
void ofApp::draw(){

    ofVec2f x;
    kf.get(x.getPtr(), 2);
    ofDrawCircle(x, 10);
}
