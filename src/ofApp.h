#pragma once

#include "ofMain.h"
#include "ofxKinectV2.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofxOsc.h"
#include "ofxOscBundle.h"


#define SERVER_PORT          8888
#define SERVER_IP_ADDR       "192.168.179.3"

#define CORAL_START_ADDRESS  "/coral/start"
#define CORAL_END_ADDRESS    "/coral/end"
#define CORAL_X_ADDRESS      "/coral/x"
#define CORAL_Z_ADDRESS      "/coral/z"
#define CORAL_GO_ADDRESS     "/coral/option"

#define MAYBE_START_ADDRESS "/maybe/start"
#define MAYBE_END_ADDRESS   "/maybe/end"
#define MAYBE_X_ADDRESS     "/maybe/x"
#define MAYBE_Y_ADDRESS     "/maybe/y"
#define MAYBE_GO_ADDRESS    "/maybe/option"

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
    
        void exit();

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
    
        void ringButtonPressed();
    
        ofxPanel gui;
        ofxButton ringButton;
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
    
        ofxLabel coralCount;
    
    ofxLabel coralValue1;
    ofxLabel coralValue2;
    ofxLabel coralValue3;
    ofxLabel coralValue4;
    ofxLabel coralValue5;
    ofxLabel coralValue6;
    ofxLabel coralValue7;
    ofxLabel coralValue8;
    ofxLabel coralValue9;
    ofxLabel coralValue10;
    ofxLabel coralValue11;
    ofxLabel coralValue12;
    ofxLabel coralValue13;
    ofxLabel coralValue14;
    ofxLabel coralValue15;
    ofxLabel coralValue16;
    ofxLabel coralValue17;
    ofxLabel coralValue18;
    ofxLabel coralValue19;
    ofxLabel coralValue20;
    
    ofxLabel message1;
    ofxLabel message2;
    ofxLabel message3;
    
        ofSoundPlayer ring;
};
