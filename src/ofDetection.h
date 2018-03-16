#pragma once

#include "ofMain.h"
#include "ofxCv.h"  
#include "ofxOsc.h"
#include "ofxOpenCv.h"
#include "ofxGUI.h"

#define SERVER_IP_ADDR    "192.168.0.2"
#define SERVER_PORT       8888

#define HOST "localhost"
//#define HOST "10.0.1.24"
#define PORT 12345

//#define HOST_TO_L "localhost"


#define MAXSENDSIZE 400
	
#define _USE_BLACKMAGIC
#define _USE_LIVE_VIDEO
#define BLACKMAGIC_W 1920
#define BLACKMAGIC_H 1080
#define BLACKMAGIC_FPS 30

#define WEB_CAM_W 1920
#define WEB_CAM_H 1080
#define TEST_VIDEO_W 1920
#define TEST_VIDEO_H 1080
#define CAPTURE_W 1280
#define CAPTURE_H 720




enum t_DetectMode{
    DET_MODE_GRAY = 0,
    DET_MODE_BLUE,
    DET_MODE_GREEN,
    DET_MODE_RGDIFF,
    DET_MODE_RBDIFF,
    DET_MODE_GBDIFF,
    DET_MODE_GBMIX,
    DET_MODE_GBDIFFMIX,
    DET_MODE_DEFAULT,
};

class ofDetection{
public:
    void setup();
    void setPixels(ofPixels _pixels);
    void setColorPixels(ofPixels _pixels);
    ofPixels getPixels();
    void update();
    void draw();
    void toggleImage();
    void keyPressed(int key);
    void saveParam();
    void loadParam();
    void rotateDetectOn(bool _b);
    void sendOSC(bool _b);
    void sendPosOSC(int x,int y);
    void setGradVarticle(int tMin,int tMax,int bMin,int bMax);
    
    bool bHideGui = false;
    void initAllocate(int w,int h);
    t_DetectMode detectMode;
    bool b_OscActive;
    bool b_Contrast;

    int circlePointX;
    int circlePointY;

private:
    bool b_RotateDetectOn;
    ofTrueTypeFont font;
    ofxOscSender sender;
    int sendhistory[MAXSENDSIZE];
    
    float threshold;
    ofxCv::ContourFinder contourFinder;
    
    ofxCvColorImage         colorImg;

    ofxCvGrayscaleImage 	grayImage;
    ofxCvGrayscaleImage 	grayImageThr;
    void areaChanged(int &val);
    void valChanged(int &val);
    
    ofParameter<int>  radMin;
    ofParameter<int>  radMax;
    ofParameter<int>  th;
    ofParameter<int>  _th;
    ofParameter<int>  hDetectThrS;
    ofParameter<int>  hDetectThrV;

    ofParameter<int>  ptMin;
    ofParameter<int>  ptMax;
    ofParameter<int>  pbMin;
    ofParameter<int>  pbMax;
    ofParameter<float> pbMixBalanceGBDiff;
    ofParameter<float> pbMixBalanceGBSum;
    ofParameter<int>  pbMixGBThr;
    ofParameter<float> pbRotateSpeedThr;
    ofParameter<int>  pbRotateSumThr;

    //ofParameter<int>  histscale;
    //ofParameter<int>  detectSpeedMin;
    //ofParameter<int>  detectSpeedMax;
    
    //検出エリアパラメータ 百分率
    /*
     ofParameter<int>  detectAreaTop;
     ofParameter<int>  detectAreaBottom;
     ofParameter<int>  detectAreaTopR;
     ofParameter<int>  detectAreaTopL;
     ofParameter<int>  detectAreaBottomR;
     ofParameter<int>  detectAreaBottomL;
    int areaTop,areaBottom,areaTopR,areaTopL,areaBottomR,areaBottomL;
    */
    
    ofxPanel gui;
    int i_ShowMode;
    bool bClearLog;
    
    ofxCvColorImage rgb, hsv;
    ofxCvGrayscaleImage r, g, b;
    ofxCvGrayscaleImage h, s, v;
    
    ofxCvGrayscaleImage rFull, gFull, bFull, gbFull;
    ofxCvGrayscaleImage grayDiff;

    vector<float> histogramH, histogramS, histogramV;
    
    ofImage red,green,blue,redd,redl,redn;
    
    int i_tMin,i_tMax,i_bMin,i_bMax;
    vector<ofVec3f> vf_RotateCheck;
    
};
