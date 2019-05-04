#include "world.h"
#include "../ofApp.h"

World::~World() {
	delete world;
	//TODO: clear anything thats referencing things in the world!
	for (WorldTickable* ptr : objects) {
		delete ptr;
	}
}

void World::tick() {
	world->Step(dt, pIterations, vIterations);
	for (WorldTickable* tickable : objects) {
		tickable->tick(dt);
	}
	for (WorldTickable* tickable : objects) {
		tickable->postTick(dt);
	}
}

vector<drawCommand> World::draw() {
	vector<drawCommand> allDrawCommands; // TODO: Optimize this to avoid reallocating memory every time
	for (WorldTickable* tickable : objects) {
		tickable->addDrawCommands(allDrawCommands);
	}
	return allDrawCommands;
}

void World::addBody(WorldTickable* tickable) {
	b2Body* body = tickable->createBody(world);
	objects.push_back(tickable);
}

void World::BeginContact(b2Contact* contact) {
	
}

void World::EndContact(b2Contact* contact) {
	
}

void World::PreSolve(b2Contact* contact, const b2Manifold* oldManifold) {
	
}

void World::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {
	b2Fixture* a = contact->GetFixtureA();
	b2Fixture* b = contact->GetFixtureB();
	
	WorldTickable* tickableA = static_cast<WorldTickable*>(a->GetUserData());
	WorldTickable* tickableB = static_cast<WorldTickable*>(b->GetUserData());
	if (tickableA->getTypeID() == 1) {
		CollidableBody* obj = static_cast<CollidableBody*>(tickableA);
		obj->handleCollision(contact, impulse, -1, dt);
	}
	if (tickableB->getTypeID() == 1) {
		CollidableBody* obj = static_cast<CollidableBody*>(tickableB);
		obj->handleCollision(contact, impulse, 1, dt);
	}
}