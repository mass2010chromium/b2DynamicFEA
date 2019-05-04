#pragma once
#include "worldTickable.h"
#include <iostream>
#include <vector>
#include "Box2D/Box2D.h"

using std::vector;

struct drawCommand;

class TwoDCircle : public WorldTickable {
	public:
		// Center x, center y, width, height
		TwoDCircle(double x, double y, double radius, double density=0, 
				double friction=0, double restitution=0) : 
				radius(radius), pos(x, y), density(density), 
				friction(friction), restitution(restitution), body(nullptr), fixture(nullptr) {
			
		}
		
		~TwoDCircle() {
			
		}
		
		virtual b2Body* createBody(b2World* world);
		virtual void tick(double dt);
		
		virtual b2Body* getBody() {
			return body;
		}
		
		virtual void addDrawCommands(vector<drawCommand>& commands);
		
	private:
		b2Body* body;
		b2Fixture* fixture;
		b2Vec2 pos;
		double radius;
		double density;
		double friction;
		double restitution;
};