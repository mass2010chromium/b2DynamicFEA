#include "ofApp.h"

using std::vector;

// b2RevoluteJoint* motorizedJoint = nullptr;
// DeformingBox* box = nullptr;
// DeformingBox* box2 = nullptr;
// DeformingBox* box3 = nullptr;
TwoDCircle* drop = nullptr;
TwoDCircle* shoot = nullptr;
DeformingBeam* beam = nullptr;
DeformingBeam* beam2 = nullptr;
DeformingBeam* beam3 = nullptr;
DeformingBeam* beam4 = nullptr;
// DeformingBeam* beam5 = nullptr;

bool deforming = false;
bool deforming2 = false;

//--------------------------------------------------------------
void ofApp::setup() {
	
    ofSetFrameRate(60);
	
	world = new World();
	height = 0;
	scaleFactor = 60;
	TwoDBox* ground = new TwoDBox(0, 2, 100, 2, 0, 0.1, 0);
	
	world->addBody(ground);
	
	// TwoDBox* smolBox = new TwoDBox(3, 1, 2, 2);
	// TwoDBox* smolBox2 = new TwoDBox(6, 2, 2, 2);
	TwoDBox* smolBox3 = new TwoDBox(11, 8, 1, 1);
	// TwoDCircle* smolBox4 = new TwoDCircle(2, 5, .5, 0.5, 0.125, 0.75);
	// TwoDCircle* smolBox5 = new TwoDCircle(4, 5, .5, 0.2, 0.0625, 0.75);
	// TwoDCircle* smolBox6 = new TwoDCircle(6, 5, .5, 0.1, 0.0625, 0.75);
	
	// box = new DeformingBox(b2Vec2(8,6), 1, 1, 100, 0.01);
	// box->setDrawColor(ofColor(255, 0, 0));
	// world->addBody(box);
	// box2 = new DeformingBox(b2Vec2(7.25,7.5), 1, 1, 100, 0.01);
	// box2->setDrawColor(ofColor(0, 255, 0));
	// world->addBody(box2);
	// box3 = new DeformingBox(b2Vec2(6.5,6), 1, 1, 100, 0.01);
	// box3->setDrawColor(ofColor(0, 0, 255));
	// world->addBody(box3);
	
	beam = new DeformingBeam(ofColor(100, 100, 100), b2Vec2(1, 5.5), 16, 3, 0.5, 1);
	world->addBody(beam);
	
	DeformingBeam* tmp = new DeformingBeam(ofColor(100, 100, 100), b2Vec2(1, 9.5), 3, 8, 0.5, 1);
	world->addBody(tmp);
	
	for (int i = 0; i < 3; i++) {
		b2Vec2 anchor(1.25 + 0.5 * i, 13.5);
		b2WeldJointDef jd;
		jd.Initialize(tmp->getBoxAt(7, i)->getBody(), ground->getBody(), anchor);
		world->createJoint(&jd);
	}
	
	beam2 = new DeformingBeam(ofColor(100, 100, 100), b2Vec2(13, 5.5), 6, 3, 0.5, 1);
	world->addBody(beam2);
	
	beam3 = new DeformingBeam(ofColor(100, 100, 100), b2Vec2(13, 9.5), 6, 8, 0.5, 1);
	world->addBody(beam3);
	
	beam4 = new DeformingBeam(ofColor(100, 100, 100), b2Vec2(18, 8.5), 20, 3, 0.5, 1);
	world->addBody(beam4);
	
	// beam5 = new DeformingBeam(ofColor(100, 100, 100), b2Vec2(18, 7.5), 20, 3, 0.5, 1);
	// world->addBody(beam5);
	
	for (int i = 0; i < 1; i++) {
		b2Vec2 anchor(1, 5.75 + i * 0.5);
		b2Vec2 anchor2(9, 5.75 + i * 0.5);
		b2WeldJointDef jd;
		jd.Initialize(beam->getBoxAt(i, 0)->getBody(), ground->getBody(), anchor);
		// jd.enableMotor = false;
		world->createJoint(&jd);
		b2WeldJointDef jd2;
		jd2.Initialize(beam->getBoxAt(i, 15)->getBody(), ground->getBody(), anchor2);
		// jd2.enableMotor = false;
		world->createJoint(&jd2);
		// b2PrismaticJointDef jd2;
		// jd2.Initialize(beam->getBoxAt(i, 9)->getBody(), ground->getBody(), anchor2, b2Vec2(1.0f, 0.0f));
		// world->createJoint(&jd2);
	}
	
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 2; j++) {
			b2Vec2 anchor(13+0.5*j, 5.75 + i * 0.5);
			b2WeldJointDef jd;
			jd.Initialize(beam2->getBoxAt(i, j)->getBody(), ground->getBody(), anchor);
			world->createJoint(&jd);
		}
	}
	
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 2; j++) {
			b2Vec2 anchor(13+0.5*j, 9.75 + i * 0.5);
			b2WeldJointDef jd;
			jd.Initialize(beam3->getBoxAt(i, j)->getBody(), ground->getBody(), anchor);
			world->createJoint(&jd);
		}
	}
	
	// for (int i = 0; i < 3; i++) {
		// for (int j = 0; j < 5; j++) {
			// b2Vec2 anchor(18+0.5*j, 5.75 + i * 0.5);
			// b2WeldJointDef jd;
			// jd.Initialize(beam4->getBoxAt(i, j)->getBody(), ground->getBody(), anchor);
			// world->createJoint(&jd);
		// }
	// }
	
	b2Vec2 anchor(18, 8.75);
	b2Vec2 anchor2(28, 8.75);
	b2WeldJointDef jd;
	jd.Initialize(beam4->getBoxAt(0, 0)->getBody(), ground->getBody(), anchor);
	world->createJoint(&jd);
	b2WeldJointDef jd2;
	jd2.Initialize(beam4->getBoxAt(0, 19)->getBody(), ground->getBody(), anchor2);
	world->createJoint(&jd2);
	
	// for (int i = 0; i < 3; i++) {
		// b2Vec2 anchor(18, 7.75);
		// b2Vec2 anchor2(28, 7.75);
		// b2WeldJointDef jd;
		// jd.Initialize(beam5->getBoxAt(i, 0)->getBody(), ground->getBody(), anchor);
		// world->createJoint(&jd);
		// b2WeldJointDef jd2;
		// jd2.Initialize(beam5->getBoxAt(i, 19)->getBody(), ground->getBody(), anchor2);
		// world->createJoint(&jd2);
	// }
	
	// world->addBody(smolBox);
	// world->addBody(smolBox2);
	// world->addBody(smolBox3);
	// world->addBody(smolBox4);
	// world->addBody(smolBox5);
	// world->addBody(smolBox6);
	
	
	// b2Body* body1 = smolBox4->getBody();
	// b2Body* body2 = smolBox5->getBody();
	// b2Body* body3 = smolBox6->getBody();
	
	// b2RevoluteJointDef rjd;

	// rjd.Initialize(body1, body2, b2Vec2(4, 5));
	// rjd.motorSpeed = -1.0f * b2_pi;
	// rjd.maxMotorTorque = 10000.0f;
	// rjd.enableMotor = false;
	// // rjd.lowerAngle = -0.25f * b2_pi;
	// // rjd.upperAngle = 0.5f * b2_pi;
	// // rjd.enableLimit = true;
	// rjd.collideConnected = false;
	
	// motorizedJoint = (b2RevoluteJoint*)world->createJoint(&rjd);
	
	
	// b2PrismaticJointDef pjd;
	// pjd.Initialize(body2, body3, b2Vec2(4, 5), b2Vec2(1.0f, 0.0f));
	// // pjd.motorSpeed = 10.0f;
	// // pjd.maxMotorForce = 10000.0f;
	// pjd.enableMotor = false;
	// // pjd.lowerTranslation = 0.0f;
	// // pjd.upperTranslation = 20.0f;
	// pjd.enableLimit = false;

	// world->createJoint(&pjd);
	
	// b2DistanceJointDef distance1;
	// distance1.Initialize(body1, body2, body1->GetPosition(), body2->GetPosition());
	// distance1.collideConnected = true;
	// distance1.frequencyHz = 0.0f;
	// distance1.dampingRatio = 1.0f;
	// world->createJoint(&distance1);
	
	// b2DistanceJointDef distance2;
	// distance2.Initialize(body2, body3, body2->GetPosition(), body3->GetPosition());
	// distance2.collideConnected = true;
	// distance2.frequencyHz = 0.0f;
	// distance2.dampingRatio = 1.0f;
	// world->createJoint(&distance2);
	
}

//--------------------------------------------------------------
void ofApp::exit() {
	delete world;
}

//--------------------------------------------------------------
void ofApp::update(){
	for (int i = 0; i < 20; i++) {
		if (deforming) {
			beam->getBoxAt(2, 7)->applyForceCenter(b2Vec2(0, -2400));
			beam->getBoxAt(2, 8)->applyForceCenter(b2Vec2(0, -2400));
		}
		if (deforming2) {
			beam2->getBoxAt(2, 5)->applyForceCenter(b2Vec2(0, -2400));
			beam3->getBoxAt(7, 5)->applyForceCenter(b2Vec2(0, -2400));
		}
		world->tick();
	}
}

//--------------------------------------------------------------
// Drawing helper methods

// Params: (int) hex
void setColor(ofApp* app, const vector<double>& params) {
	ofSetColor(ofColor::fromHex(int(params[0])));
}

// Params: x, y, radius
void drawCircle(ofApp* app, const vector<double>& params) {
	vector<double> scaled = scaleVector(app->scaleFactor, params);
	ofDrawCircle(int(scaled[0]), int(-scaled[1] + app->height), int(scaled[2]));
}

// Params: x, y, width, height
void drawRectangle(ofApp* app, const vector<double>& params) {
	vector<double> scaled = scaleVector(app->scaleFactor, params);
	ofDrawRectangle(int(scaled[0]), int(-scaled[1] - scaled[3] + app->height), 
		int(scaled[2]), int(scaled[3]));
}

// Params: x1, y1, x2, y2
void drawLine(ofApp* app, const vector<double>& params) {
	vector<double> scaled = scaleVector(app->scaleFactor, params);
	ofDrawRectangle(int(scaled[0]), int(-scaled[1] + app->height), 
		int(scaled[2]), int(-scaled[3] + app->height));
}

void drawPolygon(ofApp* app, const vector<double>& params) {
	vector<double> scaled = scaleVector(app->scaleFactor, params);
	ofSetPolyMode(OF_POLY_WINDING_NONZERO);
	ofBeginShape();
	for (int i = 0; i < scaled.size() - 1; i += 2) {
		ofVertex(int(scaled[i]), int(-scaled[i + 1] + app->height));
	}
	ofEndShape();
}

vector<double> scaleVector(double scaleFactor, vector<double> input) {
	for (int i = 0; i < input.size(); i++) {
		input[i] = input[i] * scaleFactor;
	}
	return input;
}

//--------------------------------------------------------------
void ofApp::draw(){
	vector<drawCommand> commands = world->draw();
	for (drawCommand command : commands) {
		command.app = this;
		command.draw();
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key) {
		case ' ':
			// for (int i = 0; i < 1000/60; i++) {
				// if (deforming) box->applyForceSide(1, 0, 15);
				// world->tick();
			// }
			// std::cout << "TICK4" << std::endl;
			
			if (drop) {
				world->deleteBody(drop);
				drop = nullptr;
			}
			else {
				drop = new TwoDCircle(21, 10.5, .5, 300, 0.125, 0);
				world->addBody(drop);
			}
			break;
		case ';':
			// if (deforming) box->applyForceSide(1, 0, 15);
			// world->tick();
			// std::cout << "TICK" << std::endl;
			break;
		case 'n':
			deforming = !deforming;
			break;
		case 'm':
			deforming2 = !deforming2;
			break;
		case 'x':
			if (shoot) {
				world->deleteBody(shoot);
				shoot = nullptr;
			}
			else {
				shoot = new TwoDCircle(7, 12.5, .5, 50, 0.125, 0);
				world->addBody(shoot);
				shoot->getBody()->SetLinearVelocity(b2Vec2(-8, 0));
			}
			break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
	switch (key) {
		case 'n':
			// deforming = false;
			break;
	}
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
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
	height = h;
	width = w;
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
