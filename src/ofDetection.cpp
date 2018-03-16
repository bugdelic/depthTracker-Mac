#include "ofDetection.h"

using namespace ofxCv;
using namespace cv;

void ofDetection::setup() {
    
    grayImage.allocate(WEB_CAM_W,WEB_CAM_H);
    grayImageThr.allocate(WEB_CAM_W,WEB_CAM_H);

    gui.setup("panel");
    gui.add(ptMin.set("grad top Min", 0,-100,255));
    gui.add(ptMax.set("grad top Max", 255,0,500));
    gui.add(pbMin.set("grad bottom Min", 0,-100,255));
    gui.add(pbMax.set("grad bottom Max", 255,0,500));

    gui.add(radMin.set("radMin", 1,1,10));
    gui.add(radMax.set("radMax", 10,3,30));
    gui.add(th.set("contourFinder detection Thr", 200,0,255));             //cv側の検出のthreshold(2値化しないとき)
    gui.add(_th.set("Thr for binarization", 200,0,255));         //2値化のためのthreshold
    gui.add(hDetectThrS.set("H detect thr S", 128,0,255));
    gui.add(hDetectThrV.set("H detect thr V", 128,0,255));
    gui.add(pbMixBalanceGBDiff.set("MixBalanceGBDiff", 3.0, 0.001, 3.0));
    gui.add(pbMixBalanceGBSum.set("MixBalanceGBSum", 0.5, 0.001, 3.0));
    gui.add(pbMixGBThr.set("MixGBThr", 200,0,255));
    gui.add(pbRotateSpeedThr.set("RotateSpeedThr", 6.0, 0.0, 30.0));
    gui.add(pbRotateSumThr.set("RotateSumThr", 200,0,255));
    
    gui.setPosition(0, ofGetHeight()/2);
    
    //gui.add(histscale.set("histscale", 10,3,50));
    //gui.add(detectSpeedMin.set("detectSpeedMin", 4,1,30));
    //gui.add(detectSpeedMax.set("detectSpeedMax", 30,1,30));
    
    contourFinder.setMinAreaRadius(radMin);
    contourFinder.setMaxAreaRadius(radMax);
    contourFinder.setThreshold(th);
    setGradVarticle(ptMin,ptMax,pbMin,pbMax);

    // wait for half a frame before forgetting something
    contourFinder.getTracker().setPersistence(15);          //見失っても覚えててくれるパラメータ
    // an object can move up to 100 pixels per frame
    contourFinder.getTracker().setMaximumDistance(100);     //横振りとるために増やしました
    
    ptMin.addListener(this, &ofDetection::valChanged);
    ptMax.addListener(this, &ofDetection::valChanged);
    pbMin.addListener(this, &ofDetection::valChanged);
    pbMax.addListener(this, &ofDetection::valChanged);

    radMin.addListener(this, &ofDetection::valChanged);
    radMax.addListener(this, &ofDetection::valChanged);
    th.addListener(this, &ofDetection::valChanged);
    //histscale.addListener(this, &ofDetection::valChanged);
    
    //osc sender
    i_ShowMode = 0;
    bHideGui = true;
    
    red.load("red.png");
    green.load("green.png");
    blue.load("blue.png");
    redl.load("redl.png");
    redn.load("redl.png");
    redd.load("redd.png");

    sender.setup(SERVER_IP_ADDR, SERVER_PORT);
    
    detectMode = DET_MODE_GRAY;
    b_OscActive = true;
    b_Contrast = false;
    circlePointX = 0;
    circlePointY = 0;
}

void ofDetection::initAllocate(int w,int h){
    rFull.allocate(w,h);
    gFull.allocate(w,h);
    bFull.allocate(w,h);
    gbFull.allocate(w,h);
    grayDiff.allocate(w, h);
}


void ofDetection::areaChanged(int &val){
    bClearLog=true;
}

void ofDetection::valChanged(int &val){
    contourFinder.setMinAreaRadius(radMin);
    contourFinder.setMaxAreaRadius(radMax);
    contourFinder.setThreshold(th);
    setGradVarticle(ptMin,ptMax,pbMin,pbMax);
}

void ofDetection::setPixels(ofPixels _pixels){
    grayImage.setFromPixels(_pixels);
}
void ofDetection::setColorPixels(ofPixels _pixels){
    colorImg.setFromPixels(_pixels);
    int width,height;
    width = _pixels.getWidth();
    height = _pixels.getHeight();
    cout << width << height << endl;

    switch(detectMode){
        case DET_MODE_GRAY:
        {
            grayImage = colorImg;
            break;
        }
        case DET_MODE_BLUE:
        {
            bFull.setFromPixels(_pixels.getChannel(2));
            grayImage = bFull;
            break;
        }
        case DET_MODE_GREEN:
        {
            gFull.setFromPixels(_pixels.getChannel(1));
            grayImage = gFull;
            break;
        }
        case DET_MODE_RGDIFF:
        {
            rFull.setFromPixels(_pixels.getChannel(0));
            gFull.setFromPixels(_pixels.getChannel(1));
            grayDiff.absDiff(rFull, gFull);
            grayImage = grayDiff;
            break;
        }
        case DET_MODE_RBDIFF:
        {
            rFull.setFromPixels(_pixels.getChannel(0));
            bFull.setFromPixels(_pixels.getChannel(2));
            grayDiff.absDiff(rFull, bFull);
            grayImage = grayDiff;
            break;
        }
        case DET_MODE_GBDIFF:
        {
            gFull.setFromPixels(_pixels.getChannel(1));
            bFull.setFromPixels(_pixels.getChannel(2));
            grayDiff.absDiff(gFull, bFull);
            grayImage = grayDiff;
            break;
        }
        case DET_MODE_GBMIX:
        {
            gFull.setFromPixels(_pixels.getChannel(1));
            bFull.setFromPixels(_pixels.getChannel(2));
            cvAddWeighted(gFull.getCvImage(), 0.5, bFull.getCvImage(),0.5, 0, gbFull.getCvImage());
            gbFull.threshold(pbMixGBThr);
            grayImage = gbFull;
            break;
        }
        case DET_MODE_GBDIFFMIX:
        {
            gFull.setFromPixels(_pixels.getChannel(1));
            bFull.setFromPixels(_pixels.getChannel(2));
            cvAddWeighted(gFull.getCvImage(), 0.5, bFull.getCvImage(),0.5, 0, gbFull.getCvImage());
            gbFull.threshold(pbMixGBThr);
            grayDiff.absDiff(gFull, bFull);
            cvAddWeighted(gbFull.getCvImage(), pbMixBalanceGBSum/(pbMixBalanceGBSum+pbMixBalanceGBDiff), grayDiff.getCvImage(),pbMixBalanceGBDiff/(pbMixBalanceGBSum+pbMixBalanceGBDiff), 0, grayImage.getCvImage());
            break;
        }
        case DET_MODE_DEFAULT:
        {
            grayImage = colorImg;
            break;
        }
    }
}

ofPixels ofDetection::getPixels(){
    return grayImage.getPixels();
}

void ofDetection::setGradVarticle(int tMin,int tMax,int bMin,int bMax){
    i_tMin = tMin;
    i_tMax = tMax;
    i_bMin = bMin;
    i_bMax = bMax;
}


void ofDetection::update() {
    ofPixels grayImagePixel;
    int w,h;
    w = grayImage.getWidth();
    h = grayImage.getHeight();
    grayImagePixel.allocate(w, h, OF_IMAGE_GRAYSCALE);
    unsigned char *data = grayImagePixel.getData();
    unsigned char *data2 = grayImage.getPixels().getData();
    int _Min,_Max,_H;
    for(int i=0;i<h;i++){
        _Min = int((i_tMin * (h-i)+i_bMin * i)/h);
        _Max = int((i_tMax * (h-i)+i_bMax * i)/h);
        _H = _Max - _Min;
        for(int j=0;j<w;j++){
            data[i*w+j] = int(ofClamp( 255 * (data2[i*w+j] - _Min) / _H, 0, 255));
        }
    }
    grayImage.setFromPixels(grayImagePixel);
    grayImageThr = grayImage;
    grayImageThr.threshold(_th);
    grayImageThr.invert();
    contourFinder.findContours(grayImageThr);
    
}

void ofDetection::sendPosOSC(int x,int y){
    ofxOscMessage m;
    m.setAddress("/kinect/track2");
    //char x_;
    //char y_;
    //x_ = (char)(int)(255*x/ofGetWidth());
    //y_ = (char)(int)(255*y/ofGetHeight());
    //m.addCharArg(x_);
    //m.addCharArg(y_);
    cout << "detect send to "<< x - circlePointX <<":"<< y - circlePointY << endl;
    m.addIntArg(x - circlePointX);
    m.addIntArg(y - circlePointY);
    m.addIntArg(1);
    sender.sendMessage(m);
}


void ofDetection::draw() {
    RectTracker& tracker = contourFinder.getTracker();
    ofSetColor(255);
    
    ofPushMatrix();
    ofTranslate(0, ofGetHeight()/2);
    ofScale(0.5, 0.5);
    switch(i_ShowMode){
        case 0:
            grayImage.draw(0, 0);
            //colorImg.draw(0,0);
            break;
        case 1:
            grayImageThr.draw(0, 0);
            break;
        default:
            break;
    }
    
    contourFinder.draw();

    cout <<contourFinder.size()<<endl;
    for(int i = 0; (i < contourFinder.size()) and i<100 ; i++) {
        ofPoint center = toOf(contourFinder.getCenter(i));
        int label = contourFinder.getLabel(i);
        cv::Rect rect = contourFinder.getBoundingRect(i);
        ofPushMatrix();
        
        ofTranslate(center.x, center.y);
        ofPushStyle();
        ofFill();
        //ofDrawCircle(0, 0, 20);
        ofPopStyle();
        
        string msg ;
        msg = ofToString(label);
        msg += ":";
        msg += ofToString(tracker.getAge(label));
        bool b_Send = false;
        if(b_OscActive and tracker.getAge(label)==1){
            b_Send=true;
        }
        if(b_Send){
            sendPosOSC(center.x,center.y);
            
            /*ofxOscMessage m;
            m.setAddress("/mouse/position");
            char x_;
            char y_;
            x_ = (char)(int)(255*center.x/grayImageThr.getWidth());
            y_ = (char)(int)(255*center.y/grayImageThr.getHeight());
            m.addCharArg(x_);
            m.addCharArg(y_);
            //m.addCharArg((char)score);
            sender.sendMessage(m);*/
        }
        ofScale(5, 5);
        ofSetColor(0, 255, 0);
        ofDrawBitmapString(msg, 0, 0);
        
        ofPopMatrix();
    }
    
    ofPopMatrix();
    
    
    if(!bHideGui){
        gui.draw();
    }
}

void ofDetection::rotateDetectOn(bool _b){
    b_RotateDetectOn = _b;
}

void ofDetection::sendOSC(bool _b){
    b_OscActive = _b;
}

void ofDetection::keyPressed(int key) {
    if(key == OF_KEY_UP){
        detectMode = t_DetectMode( min(detectMode + 1,(int)(DET_MODE_DEFAULT)));
    }
    if(key == OF_KEY_DOWN){
        detectMode = t_DetectMode( max(detectMode - 1,0));
    }
}
void ofDetection::toggleImage(){
    i_ShowMode = (i_ShowMode + 1) %2;
}
void ofDetection::saveParam(){
    gui.saveToFile("settings.xml");
}

void ofDetection::loadParam(){
    gui.loadFromFile("settings.xml");
}

