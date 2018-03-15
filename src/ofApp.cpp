#include "ofApp.h"

void ofApp::setup(){
    
    //Uncomment for verbose info from libfreenect2
    //ofSetLogLevel(OF_LOG_VERBOSE);
    
    ofBackground(30, 30, 30);
    
    //see how many devices we have.
    ofxKinectV2 tmp;
    vector <ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();
    
    ringButton.addListener(this, &ofApp::ringButtonPressed);
    gui.setup("", "settings.xml", 10, 10);
    
    //Note you don't have to use ofxKinectV2 as a shared pointer, but if you want to have it in a vector ( ie: for multuple ) it needs to be.
    
    kinect.open(deviceList[0].serial);
    gui.add(kinect.params);
    
    // GUIを作成
    // トラッキングする円の座標
    gui.add(circleResolution.setup("circle resolution", 100, 10, MAX_COLOR_HEIGHT/2));
    gui.add(circlePointX.setup("circle center x", MAX_DEPTH_WIDTH/2, 0, MAX_DEPTH_WIDTH));
    gui.add(circlePointY.setup("circle center y", MAX_DEPTH_HEIGHT/2, 0, MAX_DEPTH_HEIGHT));

    // OpenCVで探索する時の感度
    gui.add(detectThreshold.setup("cv threshold", 200, 1, 400));
    gui.add(cvMinArea.setup("cv min area", 10, 1, 200));
    gui.add(cvMaxArea.setup("cv max area", 2000, 1, 100000));
    gui.add(cvMaxTrackRect.setup("cv max track", 10, 1, MAX_CV_TRACK_RECT));
    
    // OSCでどのくらいのレートで送信するか？の設定
    gui.add(oscSendFrameCounter.setup("osc send frame%counter", 30, 10, 300));
    
    gui.add(ringButton.setup("RESET"));
    gui.add(coralCount.setup("scaned corel", ofToString(ofGetWidth())+"x"+ofToString(ofGetHeight())));
    gui.add(coralValue1.setup("corel_1", "none"));
    gui.add(coralValue2.setup("corel_2", "none"));
    gui.add(coralValue3.setup("corel_3", "none"));
    gui.add(coralValue4.setup("corel_4", "none"));
    gui.add(coralValue5.setup("corel_5", "none"));
    gui.add(coralValue6.setup("corel_6", "none"));
    gui.add(coralValue7.setup("corel_7", "none"));
    gui.add(coralValue8.setup("corel_8", "none"));
    gui.add(coralValue9.setup("corel_9", "none"));
    gui.add(coralValue10.setup("corel_10", "none"));
    gui.add(coralValue11.setup("corel_11", "none"));
    gui.add(coralValue12.setup("corel_12", "none"));
    gui.add(coralValue13.setup("corel_13", "none"));
    gui.add(coralValue14.setup("corel_14", "none"));
    gui.add(coralValue15.setup("corel_15", "none"));
    gui.add(coralValue16.setup("corel_16", "none"));
    gui.add(coralValue17.setup("corel_17", "none"));
    gui.add(coralValue18.setup("corel_18", "none"));
    gui.add(coralValue19.setup("corel_19", "none"));
    gui.add(coralValue20.setup("corel_20", "none"));
    gui.add(message1.setup("message_1", "none"));
    gui.add(message2.setup("message_2", "none"));
    gui.add(message3.setup("message_3", "none"));

    
    // OSC用
    oscSender.setup(SERVER_IP_ADDR, SERVER_PORT);
    
    // 各種メモリ確保
    if (!colorImage.isAllocated())
        colorImage.allocate(MAX_COLOR_WIDTH, MAX_COLOR_HEIGHT, OF_IMAGE_COLOR_ALPHA);
    
    if (!depthImage.isAllocated())
        depthImage.allocate(MAX_COLOR_WIDTH, MAX_COLOR_HEIGHT, OF_IMAGE_GRAYSCALE);
}

//--------------------------------------------------------------
void ofApp::update(){
    
    kinect.update();
    
    // フレーム取得
    if( kinect.isFrameNew() ){

        // それぞれ画像を取得
        depthImage.setFromPixels(kinect.getDepthPixels() );
        colorImage.setFromPixels(kinect.getRgbPixels());

        // アクセス用のポインタ
        unsigned char *dataColor = colorImage.getPixels().getData();
        unsigned char *dataDepth = depthImage.getPixels().getData();
        
        // 特定の円形内のデータのみ取得(2値化前処理)
        for (int i = 0; i < MAX_DEPTH_WIDTH; i++) {
            for (int j = 0; j < MAX_DEPTH_HEIGHT; j++) {
                int idx = j * depthImage.getWidth() + i;

                if (!isInColorCircle(i, j)) {
                    dataDepth[idx] = 0;
                }
            }
        }
        depthImage.update();

        // OpenCVで特定
        ofxCvContourFinder cvCountourFinder;
        ofxCvGrayscaleImage cvGrayScaleImage;
        cvGrayScaleImage.setFromPixels(dataDepth, depthImage.getWidth(), depthImage.getHeight());
        cvGrayScaleImage.threshold(detectThreshold);
        cvCountourFinder.findContours(cvGrayScaleImage, cvMinArea, cvMaxArea, cvMaxTrackRect, false);
        
        coralCount.setup("scaned C/M", ofToString(cvCountourFinder.nBlobs));
        
        // 特定した矩形毎に重心特定
        trackRectCenters.clear();
        
        int jjj=0;//COREL
        int kkk=0;//MESSAGE
        bool isCorel=false;
        
        for (int i = 0; i < cvCountourFinder.nBlobs; i++) {
            int x = 0, y = 0;
            
            float sss=cvCountourFinder.blobs[i].boundingRect.width*cvCountourFinder.blobs[i].boundingRect.height;
            
            if(sss<400){
                isCorel=true;
                jjj++;
            }else{
                isCorel=false;
                kkk++;
            }
            
            for (int j = 0; j < cvCountourFinder.blobs[i].pts.size(); j++) {
                x += cvCountourFinder.blobs[i].pts[j].x;
                y += cvCountourFinder.blobs[i].pts[j].y;
                
            }
            x /= cvCountourFinder.blobs[i].pts.size();
            y /= cvCountourFinder.blobs[i].pts.size();
            
            trackRectCenters.push_back(ofPoint(x, y));
            
        }
    
        // 重心データをOSCで送信(oscSendFrameCounterフレームカウンター毎に)
        if (trackRectCenters.size() > 0 && ofGetFrameNum() % oscSendFrameCounter == 0) {
            for (int i = 0; i < trackRectCenters.size(); i++) {
                // circlePointX, circlePointY を使って座標変換した方がよさげ(受信側で
                // 重心座標(x, y)と、矩形の輪郭点のカメラからの距離平均(z=大体の高さ)を送信
                
                // 距離を特定(存在しない事はありえない)
                int idx = trackRectCenters[i].y * depthImage.getWidth() + trackRectCenters[i].x;
                unsigned char *dataDepth = depthImage.getPixels().getData();
                int z = 0;
                if (isValidDepthRange(idx))
                    z = dataDepth[idx];

                // OSCで送信(x,yについては円の中心, circlePointX, circlePointYを使って平行移動)
                ofxOscMessage message;
                message.setAddress(OSC_ADDRESS);
                message.addIntArg(trackRectCenters[i].x - circlePointX);
                message.addIntArg(trackRectCenters[i].y - circlePointY);
                if (z >= 0)    message.addIntArg(z);
                
                
                if(isCorel){
                    
                    // 送信ログ
                    std::cout << "OSC Send Corel" << trackRectCenters[i].x << ":" << trackRectCenters[i].y << ":" << z << std::endl;
                    
                    // データ送信
                    oscSender.sendMessage(message);
                    //GUI描画
                    if(jjj>i){
                        
                    switch(i){
                        case 0:
                            coralValue1.setup("corel_1", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 1:
                            coralValue2.setup("corel_2", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 2:
                            coralValue3.setup("corel_3", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 3:
                            coralValue4.setup("corel_4", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 4:
                            coralValue5.setup("corel_5", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 5:
                            coralValue6.setup("corel_6", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 6:
                            coralValue7.setup("corel_7", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 7:
                            coralValue8.setup("corel_8", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 8:
                            coralValue9.setup("corel_9", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 9:
                            coralValue10.setup("corel_10", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 10:
                            coralValue11.setup("corel_11", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 11:
                            coralValue12.setup("corel_12", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 12:
                            coralValue13.setup("corel_13", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 13:
                            coralValue14.setup("corel_14", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 14:
                            coralValue15.setup("corel_15", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 15:
                            coralValue16.setup("corel_16", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 16:
                            coralValue17.setup("corel_17", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 17:
                            coralValue18.setup("corel_18", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 18:
                            coralValue19.setup("corel_19", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 19:
                            coralValue20.setup("corel_20", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                    }
                    }
                }else{
                    
                    // 送信ログ
                    std::cout << "OSC Send Mesage" << trackRectCenters[i].x << ":" << trackRectCenters[i].y << ":" << z << std::endl;
                    
                    // データ送信
                    oscSender.sendMessage(message);
                    
                    if(kkk>i){
                    //GUI描画
                    switch(i){
                        case 0:
                            message1.setup("message_1", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 1:
                            message2.setup("message_2", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                        case 2:
                            message3.setup("message_3", "x:"+ofToString(trackRectCenters[i].x - circlePointX)+" y:"+ofToString(trackRectCenters[i].y - circlePointY));
                            break;
                    }
                    }
                    
                }
            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    float dwHD = MAX_COLOR_WIDTH/4;
    float dhHD = MAX_COLOR_HEIGHT/4;
        
    float shiftY = 50 + ((10 + depthImage.getHeight()) * 0);

    ofSetColor(255, 255, 255);
    depthImage.draw(220, shiftY);
    colorImage.draw(230 + depthImage.getWidth(), shiftY, dwHD, dhHD);
    
    // メニューの表示
    if (showMenu) {
        ofSetColor(0, 0, 0);
        gui.draw();
    }

    // 重心の画面表示用
    if (trackRectCenters.size() > 0) {
        ofSetColor(255, 0, 0);
        ofFill();
        for (int i = 0; i < trackRectCenters.size(); i++) {
            ofDrawCircle(trackRectCenters[i].x + 220, trackRectCenters[i].y + shiftY, 1);
        }
    }
}

//--------------------------------------------------------------
void ofApp::exit(){
    ringButton.removeListener(this, &ofApp::ringButtonPressed);
}

//--------------------------------------------------------------
void ofApp::ringButtonPressed(){
    
    //kinect.open(deviceList[0].serial);
    kinect.close();
    //ring.play();
    ofApp::setup();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key) {
        case 'h':
            showMenu = !showMenu;
            break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}

bool ofApp::isValidDepthRange(int index)
{
    unsigned char *dataDepth = depthImage.getPixels().getData();
    if (depthImage.getHeight() * depthImage.getWidth() <= index) return false;
    return (MIN_DEPTH_DISTANCE <= dataDepth[index]) && (dataDepth[index] <= MAX_DEPTH_DISTANCE);
}

bool ofApp::isInColorCircle(int x, int y) {
    int wi = circlePointX - x;
    int yi = circlePointY - y;
    if (sqrt(wi * wi + yi * yi) <= circleResolution) {
        return true;
    } else {
        return false;
    }
}
