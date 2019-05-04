#pragma once

#include "ofMain.h"
#include "Box2D/Box2D.h"
#include "physics/world.h"
#include <iostream>
#include "physics/twoDBox.h"
#include "physics/twoDCircle.h"
#include "physics/deformingBox.h"
#include <vector>
#include "physics/deformingBeam.h"

using std::vector;

class ofApp : public ofBaseApp {

	public:
		void setup();
		void exit();
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
	
	private:
		double scaleFactor;
		int height;
		int width;
		
		World* world;
	
	friend void setColor(ofApp* app, const vector<double>& params);
	friend void drawCircle(ofApp* app, const vector<double>& params);
	friend void drawRectangle(ofApp* app, const vector<double>& params);
	friend void drawLine(ofApp* app, const vector<double>& params);
	friend void drawPolygon(ofApp* app, const vector<double>& params);
	friend vector<double> scaleVector(double scaleFactor, vector<double> input);
};


struct drawCommand {
	drawCommand(ofApp* thisApp, void (*funcptr)(ofApp*, const vector<double>&), 
			vector<double> params) {
		drawFunc = funcptr;
		drawParams = params;
		app = thisApp;
	}
	drawCommand(void (*funcptr)(ofApp*, const vector<double>&) = nullptr, 
			vector<double> params = vector<double>()) : drawCommand(nullptr, funcptr, params) {
				
	}
	
	void draw() {
		drawFunc(app, drawParams);
	}
	
	void (*drawFunc)(ofApp*, const vector<double>&);
	vector<double> drawParams;
	ofApp* app;
};