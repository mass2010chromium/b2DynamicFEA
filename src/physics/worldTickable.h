#pragma once

#include "Box2D/Box2D.h"
#include <vector>
#include <iostream>

using std::vector;

struct drawCommand;

class WorldTickable {
	
	public:	
		virtual void tick(double dt) {
			// Do nothing. Override me!
		}
		
		virtual void postTick(double dt) {
			// Do nothing. Override me!
		}

		virtual b2Body* createBody(b2World* world) = 0;
		virtual b2Body* getBody() = 0;
		
		virtual void addDrawCommands(vector<drawCommand>& commands) {
			// Pass. Add draw commands to the list
			std::cout << "bad" << std::endl;
		}
		
		virtual void deleteBodies(b2World* world) {
			world->DestroyBody(getBody());
		}
		
		virtual int getTypeID() {
			return 0;
		}
};