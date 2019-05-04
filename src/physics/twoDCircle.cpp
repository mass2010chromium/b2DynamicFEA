#include "twoDCircle.h"
#include "../ofApp.h"
#include <iostream>

struct drawCommand;

b2Body* TwoDCircle::createBody(b2World* world) {
	if (body) {
		std::cout << "Failed to create body: Already had body!" << std::endl;
		throw -1;
	}
	
	b2BodyDef definition;
	definition.position = pos;
	definition.type = b2_dynamicBody;
	body = world->CreateBody(&definition);
	b2CircleShape circle;
	circle.m_p.Set(0, 0);
	circle.m_radius = radius;
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &circle;
	fixtureDef.userData = this; // ALL USER DATA ARE WORLDTICKABLES!!!
	fixtureDef.density = density;
	fixtureDef.friction = friction;
	fixtureDef.restitution = restitution;
	fixture = body->CreateFixture(&fixtureDef);
	return body;
}

void TwoDCircle::tick(double dt) {
	pos = body->GetPosition();
	// std::cout << pos.x << "," << pos.y << ", " << density << std::endl;
}

void TwoDCircle::addDrawCommands(vector<drawCommand>& commands) {
	vector<double> drawParams {pos.x, pos.y, radius};
	commands.push_back(drawCommand(nullptr, drawCircle, drawParams));
}