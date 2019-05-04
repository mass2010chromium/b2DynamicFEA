#include "twoDBox.h"
#include "../ofApp.h"

struct drawCommand;

b2Body* TwoDBox::createBody(b2World* world) {
	if (body) {
		std::cout << "Failed to create body: Already had body!" << std::endl;
		throw -1;
	}
	
	b2BodyDef definition;
	definition.position = pos;
	body = world->CreateBody(&definition);
	b2PolygonShape box;
	box.SetAsBox(width/2, height/2);
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &box;
	fixtureDef.userData = this; // ALL USER DATA ARE WORLDTICKABLES!!!
	fixtureDef.density = density;
	fixtureDef.friction = friction;
	fixtureDef.restitution = restitution;
	fixture = body->CreateFixture(&fixtureDef);
	return body;
}

void TwoDBox::tick(double dt) {
	pos = body->GetPosition();
}

void TwoDBox::addDrawCommands(vector<drawCommand>& commands) {
	vector<double> drawParams {pos.x - width/2, pos.y - height/2, width, height};
	vector<double> colorParams {double(0xffffff)};
	commands.push_back(drawCommand(setColor, colorParams));
	commands.push_back(drawCommand(nullptr, drawRectangle, drawParams));
}