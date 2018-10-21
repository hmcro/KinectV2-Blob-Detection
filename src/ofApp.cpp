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
    grayImage.allocate(320,240);
    grayBg.allocate(320,240);
    
    // #P values for margins and padding between screens
    pad = 30;
    margin = 50;
    lw = 640;
    lh = 480;
    sw = 320;
    sh = 240;
    
    cout << "kinects.size() returns " << kinects.size() << "\n";
    
    if (kinects.size() < 1) kinected = false;
    else kinected = true;
    
    // #P To be implemented later
    // "Min & Max Distance of Depth Detection in MM"
    panel.setup("HMCRO", "settings.xml", margin, margin);
    
    
    // #P Change Kinects from Vector to single object (we will only be using one kinect)
    //Note you don't have to use ofxKinectV2 as a shared pointer, but if you want to have it in a vector ( ie: for multuple ) it needs to be.
    for(int d = 0; d < kinects.size(); d++){
        kinects[d] = shared_ptr <ofxKinectV2> (new ofxKinectV2());
        kinects[d]->open(deviceList[d].serial);
        // #P Takes parameters from the kinect object and feeds them to ofxpanel, giving us the sliders
        panel.add(kinects[d]->params);
    }
    
    // #P Boolean used to calibrate background
    bLearnBackground = true;
    
    // #P create all the parameters that we will need:
    parameters.add(threshold.set("Threshold",0,0,255));    // #P threshold for determining solid vs empty space
    parameters.add(minRectSize.set("Minimum Blob Size",0,0,1000)); // #P Min size for a rectangle around a blob
    parameters.add(maxRectSize.set("Maximum Blob Size",0,0,320*240/3)); // #P Max size for a rectangle around a blob
    parameters.add(rgbDisplay.set("Display RGB Output",2));
    parameters.add(depthDisplay.set("Display Depth Output",2));
    parameters.add(thresholdDisplay.set("Display Threshold Output",2));
    parameters.add(scaleX.set("ScaleX",2,0,4));
    
    // #P Then add them to actual visible panel
    panel.add(threshold);
    panel.add(minRectSize);
    panel.add(maxRectSize);
    panel.add(rgbDisplay);
    panel.add(depthDisplay);
    panel.add(thresholdDisplay);
    panel.add(scaleX);
    
    
    panel.add(p1.setup("Upper Left Point",  ofVec2f(40, 30),           ofVec2f(0, 0), ofVec2f(80, 60)));
    panel.add(p2.setup("Upper Right Point", ofVec2f(320-40, 30),       ofVec2f(320-80, 0), ofVec2f(320, 60)));
    panel.add(p3.setup("Lower Right Point", ofVec2f(320-40, 240-30),   ofVec2f(320-80, 240-60), ofVec2f(320, 240)));
    panel.add(p4.setup("Lower Left Point",  ofVec2f(40, 240-30),       ofVec2f(0, 240-60), ofVec2f(80, 240)));
    
    // #P Once parameters are initiated, load the values from out settings.xml file
    panel.loadFromFile("settings.xml");
    
    panel.setSize(sw, sh);
    panel.setWidthElements(sw);
    panel.setDefaultWidth(sw);
    panel.setDefaultHeight(sh);
    
    //scaleX = 2;
    
    midScreenX = ofGetWidth()/2 - grayDiff.width*scaleX/2;
    midScreenY = ofGetHeight()/2 - grayDiff.height*scaleX/2;
    
    rgbSize = 381;
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    if (kinected) {
        // #P cycling through kinect vector to be removed
        for(int d = 0; d < kinects.size(); d++){
            kinects[d]->update();
            if( kinects[d]->isFrameNew() ){
                // #P gets depth data
                texDepth[d].loadData( kinects[d]->getDepthPixels() );
                
                // #P gets rgb data, probably do not need this, or possible limit to 1fps to reduce cpu usage
                if(rgbDisplay)texRGB[d].loadData( kinects[d]->getRgbPixels() );
                
                //
                
                
                
                // #P load depth data to of pixel array
                texDepth[d].readToPixels(pixels);
                
                // #P resize pixels ahead of setting to image object (which already has memory allocated)
                // Working options for 3rd argument are OF_INTERPOLATE_BICUBIC or OF_INTERPOLATE_NEAREST_NEIGHBOR
                pixels.resize(320, 240, OF_INTERPOLATE_BICUBIC);
                
                // #P load into ofimage object
                grayImage.setFromPixels(pixels);
                
                
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
                contourFinder.findContours(grayDiff, minRectSize, maxRectSize, 10, true, true);	// find holes
            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    
    //ofDrawBitmapString("ofxKinectV2: Work in progress addon.\nBased on the excellent work by the OpenKinect libfreenect2 team\n\n-Requires USB 3.0 port ( superspeed )\n-Requires patched libusb. If you have the libusb from ofxKinect ( v1 ) linked to your project it will prevent superspeed on Kinect V2", 5, 5);
    
    
    
    if (kinected){
        
        // move all image and blob drawing to centre and scale
        ofSetHexColor(0xffffff);
        ofPushMatrix();
        ofTranslate(midScreenX, midScreenY);
        ofScale(scaleX,scaleX);
        
        //grey rectangle to put blobs in
        ofFill();
        ofSetHexColor(0x333333);
        ofDrawRectangle(0,0,320,240);
        ofSetHexColor(0xffffff);
        
        // #P draw all images on screen for  reference
        ofSetHexColor(0xffffff);
        
        //texDepth[0].draw(margin,margin);
        
        //grayBg.draw(margin,lh+pad+margin);
        
        
        if(thresholdDisplay)grayDiff.draw(0,0);
        if(depthDisplay)grayImage.draw(0,0);
        
        
        ofEnableAlphaBlending();
        ofSetColor(255,255,255,100);
        if(rgbDisplay){
            tmpX = (320-rgbSize)/2;
            tmpY = tmpX*9/16 + 30; //(240-240*9/16)/2;
            texRGB[0].draw(tmpX, tmpY, rgbSize, rgbSize*9/16);
        }
        ofDisableAlphaBlending();
        
        
        
        
        // or, instead we can draw each blob individually from the blobs vector,
        // this is how to get access to them:
        
        
        
        
        //Draw Polygon within which we'll be checking if the rectangles fall
        drawPolygon();
        
        
        for (int i = 0; i < contourFinder.nBlobs; i++){
            contourFinder.blobs[i].draw();
            if (detectionArea.inside(contourFinder.blobs[i].boundingRect.getCenter().x, contourFinder.blobs[i].boundingRect.getCenter().y)) {
                ofNoFill();
                ofSetColor(0,255,255);
                ofDrawRectangle(contourFinder.blobs[i].boundingRect.x, contourFinder.blobs[i].boundingRect.y, contourFinder.blobs[i].boundingRect.width, contourFinder.blobs[i].boundingRect.height);
            }
            else{
                ofNoFill();
                ofSetColor(100,100,0);
                ofDrawRectangle(contourFinder.blobs[i].boundingRect.x, contourFinder.blobs[i].boundingRect.y, contourFinder.blobs[i].boundingRect.width, contourFinder.blobs[i].boundingRect.height);
            }
            
            // draw over the centroid if the blob is a hole
            ofSetHexColor(0xffffff);
            if(contourFinder.blobs[i].hole){
                ofDrawBitmapString("hole",
                                   contourFinder.blobs[i].boundingRect.getCenter().x,
                                   contourFinder.blobs[i].boundingRect.getCenter().y);
            }
        }
        ofPopMatrix();
        
        //    if (contourFinder.nBlobs > 0) {
        //        ofPoint point = contourFinder.blobs[0].boundingRect.getCenter();
        //        cout << "ofPpoint point x=" << point.x << ", y=" << point.y << "\n";
        //    }
        
        // finally, a report:
        ofSetHexColor(0xffffff);
        stringstream reportStr;
        reportStr << "bg subtraction and blob detection" << endl
        << "press ' ' to capture bg" << endl
        << "threshold " << threshold << " (press: +/-)" << endl
        << "num blobs found " << contourFinder.nBlobs << ", fps: " << ofGetFrameRate();
        ofDrawBitmapString(reportStr.str(), 20, 600);
        
        
        panel.draw();
        
    }
    else {
        ofSetHexColor(0xffffff);
        stringstream reportStr;
        reportStr << "THERE IS NO KINECT CONNECTED TO THIS SYSTEM" << endl
        << "fps: " << ofGetFrameRate();
        ofDrawBitmapString(reportStr.str(), 30, 30);
        
    }
}

//--------------------------------------------------------------
void ofApp::drawPolygon(){
    
    detectionArea.clear();
    
    verts[0] = (ofVec2f)p1;
    verts[1] = (ofVec2f)p2;
    verts[2] = (ofVec2f)p3;
    verts[3] = (ofVec2f)p4;
    
    detectionArea.addVertices(verts, 4);
    detectionArea.close(); // close the shape
    //ofFill();
    ofSetColor(200,200,200);
    detectionArea.draw();
    ofSetHexColor(0xffffff);
    
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
            rgbSize ++;
            cout << "width=" << texRGB[0].getWidth() << "\nheight=" << texRGB[0].getHeight() << "\n";
            
            cout << "x=" << tmpX << "\nY=" << tmpY << "\n";
            
            break;
        case '-':
            rgbSize --;
            if (rgbSize < 0) rgbSize = 0;
            cout << "width=" << texRGB[0].getWidth() << "\nheight=" << texRGB[0].getHeight() << "\n";
            cout << "x=" << tmpX << "\nY=" << tmpY << "\nrgbSize=" << rgbSize << "\n";
            break;
        case 'd':
            cout << texDepth[0].getWidth() << "\n";
            cout << texDepth[0].getHeight() << "\n";
            cout << grayImage.getWidth() << "\n";
            cout << grayImage.getHeight() << "\n";
            cout << grayDiff.getWidth() << "\n";
            cout << grayDiff.getHeight() << "\n";
            break;
        case 's':
            panel.saveToFile("settings.xml");
            cout << "Saved to file";
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
    cout << "mouseReleased outside panel!\n";
    // #P This should save all our parameters whenever we have changed them with the mouse
    // But mouse releases aren't recorded here when they are within the panel!
    panel.saveToFile("settings.xml");
    cout << "Saved to file";
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
