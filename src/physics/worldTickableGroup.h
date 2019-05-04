#pragma once

#include <vector>
#include "worldTickable.h"

using std::vector;

struct drawCommand;

class WorldTickableGroup : public WorldTickable {
	public:

		virtual b2Body* getBody() {
			return nullptr;
		}
		
		virtual int getTypeID() {
			return 2;
		}
		
		virtual vector<WorldTickable*> getParts() = 0;
};