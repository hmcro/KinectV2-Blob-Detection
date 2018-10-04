#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    //Uncomment for verbose info from libfreenect2
    //ofSetLogLevel(OF_LOG_VERBOSE);
    
    ofBackground(30, 30, 30);
    
    //see how many devices we have. // #P TO BE REMOVED as using
    ofxKinectV2 tmp;
    vector <ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();
    
    //allocate for this many devices
    kinects.resize(deviceList.size());
    texDepth.resize(kinects.size());
    texRGB.resize(kinects.size());
    
    // #P this is needed otherwise openCV can't read data
    grayDiff.allocate(320,240);
    
    cout << "kinects.size() returns " << kinects.size() << "\n";
    
    // #P To be implemented later
    panel.setup("", "settings.xml", 10, 100);
    
    //Note you don't have to use ofxKinectV2 as a shared pointer, but if you want to have it in a vector ( ie: for multuple ) it needs to be.
    for(int d = 0; d < kinects.size(); d++){
        kinects[d] = shared_ptr <ofxKinectV2> (new ofxKinectV2());
        kinects[d]->open(deviceList[d].serial);
        // #P Takes parameters from the kinect object and feeds them to ofxpanel, giving us the sliders
        panel.add(kinects[d]->params);
    }
    
    // #P Boolean used to calibrate background
    bLearnBackground = true;
    
    // #P values for margins and padding between screens
    pad = 30;
    margin = 50;
    lw = 640;
    lh = 480;
    sw = 320;
    sh = 240;
    
    // #P currently unimplemented
    panel.loadFromFile("settings.xml");
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    // #P cycling through kinect vector to be removed
    for(int d = 0; d < kinects.size(); d++){
        kinects[d]->update();
        if( kinects[d]->isFrameNew() ){
            // #P gets depth data
            texDepth[d].loadData( kinects[d]->getDepthPixels() );
            
            // #P gets rgb data, probably do not need this, or possible limit to 1fps to reduce cpu usage
            texRGB[d].loadData( kinects[d]->getRgbPixels() );
            
            // #P load depth data to of pixel array
            texDepth[d].readToPixels(pixels);
            
            // #P load into ofimage object
            grayImage.setFromPixels(pixels);
            
            // #P resize image fit allocated
            grayImage.resize(320, 240);
            
            if (bLearnBackground == true){
                // #P stored an image of background to later comparison to current image
                grayBg = grayImage;
                bLearnBackground = false;
            }
            
            // take the abs value of the difference between background and incoming and then threshold:
            grayDiff.absDiff(grayBg, grayImage);
            // #P may be possible to speed this up with: cvThreshold(cvImage, cvImage, value, 255, type);
            // (jump to definition of 'threshold')
            grayDiff.threshold(threshold);
            
            
            
            // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
            // also, find holes is set to true so we will get interior contours as well....
            // #P Minimum and maximum sizes to be modified to be live controlled and stored by ofxPanel
            contourFinder.findContours(grayDiff, 20, (320*240)/3, 10, true);	// find holes
        }
    }
    
    
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    //ofDrawBitmapString("ofxKinectV2: Work in progress addon.\nBased on the excellent work by the OpenKinect libfreenect2 team\n\n-Requires USB 3.0 port ( superspeed )\n-Requires patched libusb. If you have the libusb from ofxKinect ( v1 ) linked to your project it will prevent superspeed on Kinect V2", 5, 5);
    
    
    
    //    lh = texDepth[0].getHeight();
    //    lw = texDepth[0].getWidth();
    //    sh = grayBg.getHeight();
    //    sw = grayBg.getWidth();
    
    // #P draw all images on screen for  reference
    ofSetHexColor(0xffffff);
    texDepth[0].draw(margin,margin);
    grayImage.draw(lw+pad,margin);
    grayBg.draw(margin,lh+pad+margin);
    grayDiff.draw(sw+margin+pad,lh+pad+margin);
    
    //grey rectangle to put blobs in
    ofFill();
    ofSetHexColor(0x333333);
    ofDrawRectangle(sw*2+pad*2+margin,lh+pad+margin,320,240);
    ofSetHexColor(0xffffff);
    
    // we could draw the whole contour finder
    //contourFinder.draw(730,500);
    
    // or, instead we can draw each blob individually from the blobs vector,
    // this is how to get access to them:
    for (int i = 0; i < contourFinder.nBlobs; i++){
        contourFinder.blobs[i].draw(sw*2+pad*2+margin,lh+pad+margin);
        
        // draw over the centroid if the blob is a hole
        ofSetColor(255);
        if(contourFinder.blobs[i].hole){
            ofDrawBitmapString("hole",
                               contourFinder.blobs[i].boundingRect.getCenter().x + 360,
                               contourFinder.blobs[i].boundingRect.getCenter().y + 540);
        }
    }
    
    // finally, a report:
    ofSetHexColor(0xffffff);
    stringstream reportStr;
    reportStr << "bg subtraction and blob detection" << endl
    << "press ' ' to capture bg" << endl
    << "threshold " << threshold << " (press: +/-)" << endl
    << "num blobs found " << contourFinder.nBlobs << ", fps: " << ofGetFrameRate();
    ofDrawBitmapString(reportStr.str(), 20, 600);
    
    
    
    
    //    for(int d = 0; d < kinects.size(); d++){
    //        float dwHD = 1920/4;
    //        float dhHD = 1080/4;
    //
    //        float shiftY = 100 + ((10 + texDepth[d].getHeight()) * d);
    //
    //                texDepth[d].draw(200, shiftY);
    //                texRGB[d].draw(210 + texDepth[d].getWidth(), shiftY, dwHD, dhHD);
    //    }
    
    panel.draw();
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    switch (key){
            // #P Press space to record a frame to calibrate background
        case ' ':
            bLearnBackground = true;
            break;
            
            // #P Allow keyboard to control threshold
        case '=':
            threshold ++;
            if (threshold > 255) threshold = 255;
            break;
        case '-':
            threshold --;
            if (threshold < 0) threshold = 0;
            break;
        case 'd':
            cout << texDepth[0].getWidth() << "\n";
            cout << texDepth[0].getHeight() << "\n";
            cout << grayImage.getWidth() << "\n";
            cout << grayImage.getHeight() << "\n";
            cout << grayDiff.getWidth() << "\n";
            cout << grayDiff.getHeight() << "\n";
            break;
    }
    
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){
    
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
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
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
