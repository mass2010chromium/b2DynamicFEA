#pragma once
#pragma once
#include "worldTickable.h"
#include <iostream>
#include <vector>
#include "Box2D/Box2D.h"

using std::vector;

struct drawCommand;

class BoxObject : public WorldTickable {
public:

	BoxObject() {}
	BoxObject(b2BodyDef bodyDef, b2FixtureDef fixtureDef) {
		set(bodyDef, fixtureDef);
	}

	~BoxObject() {}

	virtual b2Body* createBody(b2World* world);
	virtual void tick(double dt);

	virtual b2Body* getBody() {
		return body;
	}

	virtual void addDrawCommands(vector<drawCommand>& commands);

	void set(b2BodyDef bodyDef, b2FixtureDef fixtureDef) {
		this->bodyDef = bodyDef;
		this->fixtureDef = fixtureDef;
		this->fixtureDef.userData = this; // ALL USER DATA ARE WORLDTICKABLES!!!
	}

private:
	b2BodyDef bodyDef;
	b2FixtureDef fixtureDef;
	
	b2Body* body;
	b2Fixture* fixture;
};