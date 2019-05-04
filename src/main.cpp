#include "ofMain.h"
#include "ofApp.h"

//========================================================================
int main(int argc, char** argv) {
	B2_NOT_USED(argc);
	B2_NOT_USED(argv);
	ofSetupOpenGL(1800,1000,OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new ofApp());
	
	return 0;
}
