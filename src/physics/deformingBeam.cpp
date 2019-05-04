#include "deformingBeam.h"

DeformingBeam::DeformingBeam(ofColor color, b2Vec2 bottomLeft, int width, int height, double size, int id) {
	// std::cout << "ASDF" << std::endl;
	this->width = width;
	this->height = height;
	this->size = size;
	this->id = id;
	boxes = vector<DeformingBox*>();
	
	centerPos = bottomLeft;
	centerPos.x += size / 2;
	centerPos.y += size / 2;
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			DeformingBox* gridBox = new DeformingBox(b2Vec2(centerPos.x + col * size, centerPos.y + row * size), size);
			// gridBox->setDrawColor(color);
			if (row % 3 == 0) gridBox->setDrawColor(ofColor(100, 100, 100));
			if (row % 3 == 1) gridBox->setDrawColor(ofColor(150, 150, 150));
			if (row % 3 == 2) gridBox->setDrawColor(ofColor(50, 50, 50));
			gridBox->setDensity(100);
			boxes.push_back(gridBox);
		}
	}
	
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			DeformingBox* target = getBoxAt(row, col);
			target->setNeighbor(0, getBoxAt(row, col + 1));
			target->setNeighbor(1, getBoxAt(row + 1, col));
			target->setNeighbor(2, getBoxAt(row, col - 1));
			target->setNeighbor(3, getBoxAt(row - 1, col));
		}
	}
}

b2Body* DeformingBeam::createBody(b2World* world) {
	for (DeformingBox* box : boxes) {
		box->createBody(world);
		b2Filter filter = box->getFixture()->GetFilterData();
		filter.groupIndex = -id;
		box->getFixture()->SetFilterData(filter);
	}
	// for (int row = 0; row < height; row++) {
		// for (int col = 0; col < width; col++) {
			// DeformingBox* target = getBoxAt(row, col);
			// if (row < height - 1) {
				// b2WeldJointDef jd;
				// b2Vec2 myPoint = centerPos;
				// myPoint += b2Vec2(col * size, (0.5 + row) * size);
				// jd.Initialize(getBoxAt(row + 1, col)->getBody(), target->getBody(), myPoint);
				// world->CreateJoint(&jd);
			// }
			// if (col < width - 1) {
				// b2WeldJointDef jd;
				// b2Vec2 myPoint = centerPos;
				// myPoint += b2Vec2((0.5 + col) * size, row * size);
				// jd.Initialize(getBoxAt(row, col + 1)->getBody(), target->getBody(), myPoint);
				// world->CreateJoint(&jd);
			// }
		// }
	// }
	return nullptr;
}

void DeformingBeam::addDrawCommands(vector<drawCommand>& commands) {
	for (WorldTickable* t : getParts()) {
		((DeformingBox*) t)->addDrawCommands(commands);
	}
}

void DeformingBeam::tick(double dt) {
	for (WorldTickable* t : getParts()) {
		((DeformingBox*) t)->tick(dt);
	}
}

void DeformingBeam::postTick(double dt) {
	for (WorldTickable* t : getParts()) {
		((DeformingBox*) t)->postTick(dt);
	}
}

void DeformingBeam::deleteBodies(b2World* world) {
	for (WorldTickable* t : getParts()) {
		((DeformingBox*) t)->deleteBodies(world);
	}
}