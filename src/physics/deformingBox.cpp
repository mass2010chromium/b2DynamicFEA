#include "deformingBox.h"
#include "../ofApp.h"

b2Body* DeformingBox::createBody(b2World* world) {
	if (body) {
		std::cout << "Failed to create body: Already had body! @DeformingBox" << std::endl;
		throw -1;
	}
	gravity = world->GetGravity();
	b2BodyDef definition;
	definition.fixedRotation = false;
	definition.type = b2_dynamicBody;
	definition.position = center;
	body = world->CreateBody(&definition);
	b2PolygonShape box;
	box.SetAsBox(radii(0) /2, radii(1) /2);
	b2MassData* tmpData = new b2MassData();
	box.ComputeMass(tmpData, density);
	this->massData = *tmpData;
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &box;
	fixtureDef.userData = this; // ALL USER DATA ARE WORLDTICKABLES!!!
	fixtureDef.density = density;
	fixtureDef.friction = friction;
	fixtureDef.restitution = restitution;
	fixture = body->CreateFixture(&fixtureDef);
	return body;
}

void DeformingBox::setVerticesNormals() {
	b2PolygonShape* box = (b2PolygonShape*) fixture->GetShape();
	
	b2Vec2 axis2 = getAxis2();
	b2Vec2 edge1 = axis1;
	edge1 *= radii(0);
	b2Vec2 edge2 = axis2;
	edge2 *= radii(1);
	b2Vec2 corner1 = edge1 + edge2;
	b2Vec2 corner2 = edge1 - edge2;
	
	box->m_vertices[0].Set(-corner1.x, -corner1.y);
	box->m_vertices[1].Set(corner2.x, corner2.y);
	box->m_vertices[2].Set(corner1.x, corner1.y);
	box->m_vertices[3].Set(-corner2.x, -corner2.y);
	box->m_normals[0].Set(-axis2.x, -axis2.y);
	box->m_normals[1].Set(axis1.x, axis1.y);
	box->m_normals[2].Set(axis2.x, axis2.y);
	box->m_normals[3].Set(-axis1.x, -axis1.y);
}

b2Vec2 DeformingBox::getReaction(int side) {
	if (neighbors[side]) {
		DeformingBox* neighbor = (DeformingBox*) neighbors[side];
		b2Vec2 myPoint = getPoint(side);
		b2Vec2 otherPoint = neighbor->getPoint((side + 2) % 4);
		
		b2Vec2 delta = b2Vec2(otherPoint.x - myPoint.x, otherPoint.y - myPoint.y);
		delta *= youngsModulus;
		b2Vec2 normalAxis = getNormal(side);
		double internalXForce = 2 * ((neighbor->radii(side % 2) - neighbor->naturalRadii(side % 2)) / neighbor->naturalRadii(side%2)) 
				* neighbor->youngsModulus;
		b2Vec2 internalForceVec = neighbor->getNormal((side + 2) % 4);
		internalForceVec *= internalXForce;
		delta -= internalForceVec;
		double normalForce = b2Dot(delta, normalAxis);
		double shearForce = b2Dot(delta, normalAxis.Skew());
		delta *= SPRING_FACTOR;
		applyForceCenter(delta);
		b2Vec2 shearVec = normalAxis.Skew();
		shearVec *= shearForce;
		applyForceCenter(shearVec);
		// std::cout << "Me: " << center.x << ", " << center.y << std::endl;
		// std::cout << side << std::endl;
		// std::cout << "Delta: " << delta.x << ", " << delta.y << std::endl;
		// std::cout << "Me: " << myPoint.x << ", " << myPoint.y << std::endl;
		// std::cout << "Other: " << otherPoint.x << ", " << otherPoint.y << std::endl;
		applyForceSide(side, normalForce, shearForce * 10);
		
		b2Vec2 theirNormalAxis = neighbor->getNormal(side);
		b2Vec2 correction = theirNormalAxis - normalAxis;
		applyForceSide(side, 0, b2Dot(correction, normalAxis.Skew()) * shearModulus);
	}
	return b2Vec2_zero;
}

void DeformingBox::tick(double dt) {
	
	// Pre-tick
	axis1Offset = 0;
	double angleVTmp = body->GetAngularVelocity();
	if (abs(angleVTmp) > 0.0001) {
		angularVelocity += angleVTmp;
		axis1Offset += body->GetAngle();
	}
	else {
		angularVelocity = 0;
	}
	body->SetAngularVelocity(0);
	body->SetTransform( body->GetPosition(), 0 );
	
	
	// DFSTreeNode* forceTree = DFS(this);
	b2Vec2 axis2 = getAxis2();
	//TODO propagate forces and the such
	
	for (int side = 0; side < 4; side++) {
		b2Vec2 force = getReaction(side);
	}
	
	//TMP code for testing and fun
	double moment = 0;
	double shear = 0;
	
	
	b2Vec2 velocity = body->GetLinearVelocity();
	b2Vec2 linearAccel = gravity;
	
	b2Vec2 totalForce = appliedForce;
	
	vector<b2Vec2> faceNormals {axis1, axis2, -axis1, -axis2};
	for (int i = 0; i < 4; i++) {
		moment += sideForces[i] * radii(i % 2);
		b2Vec2 shearForce = faceNormals[i].Skew();
		b2Vec2 normalForce = faceNormals[i];
		shearForce *= sideForces[i];
		normalForce *= normalForces[i];
		totalForce += shearForce;
		// Even shear forces: positive sign convention,
		// odd shear forces: negative sign convention
		shear += sideForces[i] * (1 - 2 * (i % 2));
	}
	double deformInertia = massData.mass / 2;
	
	for (int i = 0; i < 2; i++) {
		double sigma = (normalForces[i] + normalForces[i + 2]) / 2;
		double internalXForce = 2 * ((radii(i) - naturalRadii(i))/naturalRadii(i)) * youngsModulus;
		double dampForce = -xsVelocity(i) * linearDamping;
		double xAcceleration = (sigma - internalXForce + dampForce) / deformInertia;
		xsVelocity(i) += xAcceleration * dt;
	}
	
	double internalShear = (b2_pi / 2 - theta) * shearModulus;
	double shearDampForce = -shearVelocity * angularDamping;
	double shearAcceleration = (shear - internalShear + shearDampForce) / deformInertia;
	
	shearVelocity += shearAcceleration * dt;
	axis1Offset += shearVelocity/2 * dt;
	
	double angularAccel = moment / (massData.I); // HACKISH FIX
	angularVelocity += angularAccel * dt;
	
	linearAccel *= dt;
	totalForce *= (dt / (massData.mass));
	velocity += linearAccel + totalForce;
	
	if (abs(velocity.x) < 0.001) {
		velocity.x = 0;
	}
	if (abs(velocity.y) < 0.001) {
		velocity.y = 0;
	}
	velocity.x -= velocity.x * 10 * dt; // Sketchy linear damp
	velocity.y -= velocity.y * 10 * dt;
	
	// std::cout << "Velocity: "<< velocity.x << ", " << velocity.y << std::endl;
	// std::cout << "Omega: "<< angularVelocity << std::endl;

	body->SetLinearVelocity(velocity);
	
	setVerticesNormals();
}

void DeformingBox::postTick(double dt) {
	
	center = body->GetPosition();
	
	theta -= shearVelocity * dt;
	axis1 = b2Mul(b2Rot(axis1Offset + angularVelocity * dt), axis1);
	if (axis1.LengthSquared() < 0.9999 || axis1.LengthSquared() > 1.0001) {
		axis1.Normalize();
	}
	
	for (int i = 0; i < 2; i++) {
		radii(i) += xsVelocity(i);
	}
	
	// std::cout << "Applied force: " << appliedForce.x << ", " << appliedForce.y << std::endl;
	resetForces();
	// std::cout << "Applied force: " << appliedForce.x << ", " << appliedForce.y << std::endl;
}

void DeformingBox::addDrawCommands(vector<drawCommand>& commands) {
	b2PolygonShape* box = (b2PolygonShape*) fixture->GetShape();
	b2Vec2 corner1 = box->m_vertices[2];
	b2Vec2 corner2 = box->m_vertices[1];
	// b2Vec2 axis2 = b2Mul(b2Rot(theta), axis1);
	// b2Vec2 edge1 = axis1;
	// edge1 *= radii(0);
	// b2Vec2 edge2 = axis2;
	// edge2 *= radii(1);
	// b2Vec2 corner1 = edge1 + edge2;
	// b2Vec2 corner2 = edge1 - edge2;
	
	vector<double> params {-corner1.x + center.x, -corner1.y + center.y,
							corner2.x + center.x,  corner2.y + center.y,
							corner1.x + center.x,  corner1.y + center.y,
						   -corner2.x + center.x, -corner2.y + center.y};
	
	vector<double> colorParams {double(color.getHex())};
	commands.push_back(drawCommand(setColor, colorParams));
	commands.push_back(drawCommand(drawPolygon, params));
}

		
void DeformingBox::handleCollision(b2Contact* contact, const b2ContactImpulse* impulse, int whoAmI, double dt) {
	
	// std::cout << "COLLIDE" << std::endl;
	// std::cout << "center point: " << center.x << ", " << center.y << std::endl;
	
	b2WorldManifold worldManifold;
	contact->GetWorldManifold( &worldManifold );
	
	int collisionPoints = contact->GetManifold()->pointCount;
	
	b2Vec2 normalVec = worldManifold.normal;
	normalVec *= whoAmI;
	b2Vec2 tangentVec = normalVec.Skew();
	
	float normalImpulses = 0;
	float tangentImpulses = 0;
	
	b2Vec2 averagePoint = b2Vec2_zero;
	
	for (int i = 0; i < collisionPoints; i++) {
		normalImpulses += impulse->normalImpulses[i];
		tangentImpulses += impulse->tangentImpulses[i];
		
		averagePoint += worldManifold.points[i];
		// std::cout << "colliding point: " << worldManifold.points[i].x << ", " << worldManifold.points[i].y << std::endl;
	}
	
	averagePoint *= (1 / float(collisionPoints));
	normalVec *= normalImpulses;
	tangentVec *= tangentImpulses;
	b2Vec2 totalForceVec = normalVec + tangentVec;
	
	// body->ApplyLinearImpulse(totalForceVec, averagePoint, true);
	
	averagePoint -= center;
	
	b2Vec2 axis2 = getAxis2();
	b2Vec2 edge1 = axis1;
	edge1 *= radii(0);
	b2Vec2 edge2 = axis2;
	edge2 *= radii(1);
	
	vector<b2Vec2> points_sides {edge1, edge2, - edge1, - edge2};
	float bestLenSQ = (averagePoint - points_sides[0]).LengthSquared();
	int best = 0;
	for (int i = 1; i < 4; i++) {
		float lenSQ = (averagePoint - points_sides[i]).LengthSquared();
		if (lenSQ < bestLenSQ) {
			bestLenSQ = lenSQ;
			best = i;
		}
	}
	b2Vec2 targetAxis = axis1;
	if (best % 2 == 1) targetAxis = axis2;
	if (best >= 2) targetAxis *= -1;
	
	// // IGNORING MOMENT DUE TO OFFSET, POSSIBLE TODO?
	float normalForce = b2Dot(totalForceVec, targetAxis);
	float shearForce = b2Dot(totalForceVec, targetAxis.Skew());
	if (shearForce >= 2) {
		std::cout << "shear: " << shearForce << std::endl;
		std::cout << "normal: " << normalForce << std::endl;
		std::cout << "Point x: " << averagePoint.x << std::endl;
		std::cout << "Point y: " << averagePoint.y << std::endl;
	}
	if (normalForce < massData.mass * 10 && shearForce < massData.mass * 10) {
		applyForceSide(best, normalForce, shearForce);
	}
	
	// b2Vec2 finalForce = targetAxis;
	// finalForce *= normalForce;
	
	
	// applyForceSide(shearForce * dt, best);
}