#pragma once

#include "deformingBox.h"

#include "Box2D/Box2D.h"
#include "worldTickableGroup.h"
#include <vector>
#include "ofMain.h"
#include <iostream>

using std::vector;

class DeformingBeam : public WorldTickableGroup {
	public:
		//TODO: Customize solid constants
		DeformingBeam(ofColor color, b2Vec2 bottomLeft, int width, int height, double size, int id);

		virtual b2Body* createBody(b2World* world);
		
		DeformingBox* getBoxAt(int row, int col) {
			if (col < 0 || row < 0 || col >= width || row >= height) {
				// std::cout << "DeformingBeam: Tried to get box out of bounds!" << std::endl;
				// thcol -1;
				return nullptr;
			}
			return boxes[col + row*width];
		}
		
		virtual vector<WorldTickable*> getParts() {
			return vector<WorldTickable*>(boxes.begin(), boxes.end());
		}
		
		virtual void addDrawCommands(vector<drawCommand>& commands);
		virtual void tick(double dt);
		virtual void postTick(double dt);
		virtual void deleteBodies(b2World* world);

	private:
		int width;
		int height;
		double size;
		int id;
		
		b2Vec2 centerPos;
		
		vector<DeformingBox*> boxes;
};