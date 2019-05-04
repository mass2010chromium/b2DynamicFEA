#pragma once
#include <vector>
#include "Box2D/Box2D.h"
#include "worldTickable.h"
#include "collidableBody.h"

const b2Vec2 DEFAULT_GRAVITY = b2Vec2(0, -9.81f);
const double DEFAULT_DT = 1.0f / 2000.0f;
const double DEFAULT_POS_ITERATIONS = 10;
const double DEFAULT_VEL_ITERATIONS = 10;

using std::vector;

struct drawCommand;

class World : b2ContactListener{
	
	public:
		// TODO: Add more constructors
		World(b2Vec2 g = DEFAULT_GRAVITY, double dt = DEFAULT_DT, 
				double pIterations = DEFAULT_POS_ITERATIONS, double vIterations = DEFAULT_VEL_ITERATIONS) : 
				world(new b2World(g)), dt(dt), pIterations(pIterations), vIterations(vIterations){
			world->SetContactListener(this);
		}

		~World();

		void tick();
		
		vector<drawCommand> draw();
		
		void addBody(WorldTickable* tickable);
		b2Joint* createJoint(b2JointDef* def) {
			return world->CreateJoint(def);
		}
		
		void deleteBody(WorldTickable* tickable) {
			for (auto it = objects.begin(); it != objects.end(); ++it) {
				if (*it == tickable) {
					objects.erase(it);
					tickable->deleteBodies(world);
					break;
				}
			}
		}

		void BeginContact(b2Contact* contact);
		void EndContact(b2Contact* contact);
		void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);
		void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);
		
	private:
		b2World* world;
		vector<WorldTickable*> objects;
		
		double dt;
		double pIterations;
		double vIterations;
};