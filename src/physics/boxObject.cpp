#include "boxObject.h"
#include "../ofApp.h"

struct drawCommand;

b2Body* BoxObject::createBody(b2World* world) {
	if (body) {
		std::cout << "Failed to create body: Already had body!" << std::endl;
		throw -1;
	}
	body = world->CreateBody(&bodyDef);
	fixture = body->CreateFixture(&fixtureDef);
	return body;
}

void BoxObject::tick(double dt) {
	// SOMEHOW LOAD POSITIONS
}

void BoxObject::addDrawCommands(vector<drawCommand>& commands) {
	
}