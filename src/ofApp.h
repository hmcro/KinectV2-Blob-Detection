#pragma once

#include "ofMain.h"
#include "ofxKinectV2.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofPolyline.h"


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
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    vector < shared_ptr<ofxKinectV2> > kinects;
    
    vector <ofTexture> texDepth;
    vector <ofTexture> texRGB;
    
    bool                    kinected = false; // boolean to prevent crashing when drawing empty graphics objects
    
    ofxCvGrayscaleImage 	grayImage;
    ofxCvGrayscaleImage     grayBg;
    ofxCvGrayscaleImage 	grayDiff;
    
    ofxCvContourFinder      contourFinder;
    
    ofxPanel                panel;
    ofParameterGroup        parameters;
    ofParameter<int>        threshold;
    ofParameter<int>        minRectSize, maxRectSize; // this is the AREA of a rectangle, i.e. number of pixels
    ofParameter<bool>       rgbDisplay;
    ofParameter<bool>       depthDisplay;
    ofParameter<bool>       thresholdDisplay;
    ofParameter<float>      scaleX;
    ofxVec2Slider           p1,p2,p3,p4;
    ofParameter<int>        frameRate;
    
    bool                    bLearnBackground;
    
    void                    drawPolygon();
    
    ofPolyline              detectionArea;
    ofPoint* verts = new ofPoint[4];
    
    int                     midScreenX, midScreenY; // coordinates of screen to draw blob rects in
    int                     rgbSize;
    float                   tmpX,tmpY;
    
    ofPixels                pixels;
    
    // image layout variables
    int                     pad;
    int                     margin;
    int                     lh, lw, sh, sw;
    
    int                     people;
    
};
