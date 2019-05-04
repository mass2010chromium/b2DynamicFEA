#pragma once

#include "worldTickable.h"

class CollidableBody : public WorldTickable {
	public:
		// whoAmI: -1 if body A, 1 if body B
		virtual void handleCollision(b2Contact* contact, 
				const b2ContactImpulse* impulse, int whoAmI, double dt) = 0;
			
		// Type ID 1: Collidables
		virtual int getTypeID() {
			return 1;
		}
};