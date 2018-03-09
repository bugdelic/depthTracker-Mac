#pragma once

#include "ofMain.h"
#include "ofxKinectV2.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofxOsc.h"


#define SERVER_PORT       8888
#define SERVER_IP_ADDR    "192.168.0.2"
#define OSC_ADDRESS          "/kinect/track"
#define MAX_CV_TRACK_RECT    100

#define MAX_COLOR_WIDTH     1920
#define MAX_COLOR_HEIGHT    1080
#define MAX_DEPTH_WIDTH     512
#define MAX_DEPTH_HEIGHT    424
#define MAX_DEPTH_DISTANCE  4500
#define MIN_DEPTH_DISTANCE  500


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
        bool isInColorCircle(int x, int y);
        bool isValidDepthRange(int index);
    
        ofxPanel gui;
        ofxIntSlider minDepth;
        ofxIntSlider baseDepth;
        ofxIntSlider circleResolution;
        ofxIntSlider circlePointX;
        ofxIntSlider circlePointY;
        ofxIntSlider detectThreshold, cvMinArea, cvMaxArea, cvMaxTrackRect;
        ofxIntSlider oscSendFrameCounter;
        bool showMenu = true;
        vector <ofPoint> trackRectCenters;
        ofImage colorImage, depthImage;
        ofxOscSender oscSender;

        ofxKinectV2 kinect;
        ofTexture texDepth;
        ofTexture texRGB;
};
