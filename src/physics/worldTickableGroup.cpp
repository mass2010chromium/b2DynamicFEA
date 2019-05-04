#include "worldTickableGroup.h"

WorldTickableGroup::~WorldTickableGroup() {
	for (WorldTickable* t : getParts()) {
		if (t) {
			delete t;
			t = nullptr;
		}
	}
}

void WorldTickableGroup::tick(double dt) {
	for (WorldTickable* t : getParts()) {
		t->tick(dt);
	}
}

void WorldTickableGroup::postTick(double dt) {
	for (WorldTickable* t : getParts()) {
		t->postTick(dt);
	}
}

void WorldTickableGroup::addDrawCommands(vector<drawCommand>& commands) {
	for (WorldTickable* t : getParts()) {
		t->addDrawCommands(commands);
	}
}

void WorldTickableGroup::deleteBodies(b2World* world) {
	for (WorldTickable* t : getParts()) {
		t->deleteBodies(world);
	}
}