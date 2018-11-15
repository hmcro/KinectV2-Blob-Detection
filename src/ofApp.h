#pragma once

#include "ofMain.h"
#include "ofxKinectV2.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofPolyline.h"
#include "ofxOsc.h"

#define HOST "localhost"
#define PORT 12345


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
    
    vector <ofTexture>      texDepth;
    vector <ofTexture>      texRGB;
    
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
    ofParameter<bool>       bLearnBackground;
    ofParameter<bool>       quietMode;
    ofParameter<float>      scaleX;
    ofxVec2Slider           p1,p2,p3,p4;
    ofParameter<int>        frameRate;
    ofParameter<int>        kinectRate;
    ofParameter<float>      nPSmoother;
 
    
    
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
    
    
    void                    peopleCounter();
    int                     nPCount;            // counter for number of blobs that are both within polygon and not 'holes'
    vector <int>            nP;                 // array of numbers for averaging
    int                     nPIndex;
    int                     nPSize;
    int                     nPTotal;
    int                     nPMin = 1;
    int                     nPMax = 1000;
    int                     nPeople;            // final smoothed number of people
    
    int                     kinCount = 0;
    
    void                    sendOSC();
    ofxOscSender            sender;
    
};
