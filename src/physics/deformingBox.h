#pragma once
#include "Box2D/Box2D.h"
#include <vector>
#include "collidableBody.h"
#include "../customDFS.h"
#include "ofMain.h"
#include <cmath>

using std::vector;
using std::abs;

struct drawCommand;

const double DEFAULT_YOUNGS_MODULUS = 50;
const double SPRING_FACTOR = 200;
const double DEFAULT_SHEAR_MODULUS = 200;
const double DEFAULT_LINEAR_DAMPING = 200;
const double DEFAULT_ANGULAR_DAMPING = 50;

const int SOLVE_NUM_ITERATIONS = 5;

class DeformingBox : public CollidableBody, GraphNode{
	public:
		DeformingBox(b2Vec2 center, double size) : DeformingBox(center, size, size) {}
		// DeformingBox(b2Vec2 center, double width, double height);
		
		// l1, l2 are box width, height
		DeformingBox(b2Vec2 center, double l1, double l2, double density=1, double friction=0, double restitution=0, 
				double youngsModulus=DEFAULT_YOUNGS_MODULUS, double shearModulus=DEFAULT_SHEAR_MODULUS, 
				double linearDamping=DEFAULT_LINEAR_DAMPING, double angularDamping=DEFAULT_ANGULAR_DAMPING, 
				ofColor color=ofColor(255, 255, 255)) : 
				center(center), radii(l1/2, l2/2), naturalRadii(l1/2, l2/2), density(density), 
				friction(friction), restitution(restitution), color(color),
				youngsModulus(youngsModulus), shearModulus(shearModulus), 
				linearDamping(linearDamping), angularDamping(angularDamping) {
			theta = b2_pi / 2;
			axis1 = b2Vec2(1, 0);
			fixMask = 0;

			neighbors = vector<GraphNode*>(4, nullptr);
			sideForces = vector<double>(4, 0.0f);
			normalForces = vector<double>(4, 0.0f);
			
			massData.mass = 0; // POSSIBLY NOT NEEDED?
			massData.I = 0;
			massData.center = b2Vec2_zero;
			
			appliedForce = b2Vec2_zero;
		
			shearVelocity = 0;
			xsVelocity = b2Vec2_zero;
			angularVelocity = 0;
			// velocity = b2Vec2_zero;
			
			body = nullptr;
			fixture = nullptr;
		}
		
		virtual b2Body* createBody(b2World* world);
		virtual void tick(double dt);
		virtual void postTick(double dt);
		
		virtual b2Body* getBody() {
			return body;
		}
		
		virtual void addDrawCommands(vector<drawCommand>& commands);

		void applyForceCenter(b2Vec2 forceVec) {
			appliedForce += forceVec;
		}
		// Follow positive moment conventions!
		void applyForceSide(int sideIndex, double normalForce, double shearForce) {
			sideForces[sideIndex] += shearForce;
			normalForces[sideIndex] += normalForce;
			b2Vec2 axis2 = b2Mul(b2Rot(theta), axis1);
			axis2 *= normalForce;
			b2Vec2 axis1T = axis1;
			axis1T *= normalForce;
			//TODO disgusting
			if (sideIndex == 0) {
				appliedForce += axis1T;
			}
			if (sideIndex == 1) {
				appliedForce += axis2;
			}
			if (sideIndex == 2) {
				appliedForce -= axis1T;
			}
			if (sideIndex == 3) {
				appliedForce -= axis2;
			}
		}
		
		
		void resetForces() {
			appliedForce.Set(0.0f, 0.0f);
			for (int i = 0; i < 4; i++) {
				sideForces[i] = 0;
				normalForces[i] = 0;
			}
			// for (double& num : sideForces) {
				// num = 0;
			// }
		}
		

		void setNeighbor(int index, DeformingBox* target) {
			neighbors[index] = target;
		}
		
		void fixVertex(int vertex) {
			fixMask |= 1 << vertex;
		}
		
		void unfixVertex(int vertex) {
			fixMask &= ~(1 << vertex);
		}
		
		const virtual vector<GraphNode*> getNeighbors() {
			return neighbors;
		}
		
		virtual void handleCollision(b2Contact* contact, 
				const b2ContactImpulse* impulse, int whoAmI, double dt);
		
		void setDrawColor(ofColor color) {
			this->color = color;
		}
		
		b2Fixture* getFixture() {
			return fixture;
		}
		
		void setDensity(double d) {
			density = d;
		}

	private:
	
		ofColor color;
	
		b2Body* body;
		b2Fixture* fixture;
		
		// 0: right (axis), 1: up, 2: left, 3: down
		vector<GraphNode*> neighbors;
		vector<double> normalForces;
		b2Vec2 appliedForce;
		vector<double> sideForces;
		
		char fixMask;
		
		double theta; // Angle from axis 1 to axis 2
		b2Vec2 axis1; // Direction of x1
		
		b2Vec2 center;
		b2Vec2 gravity;
		// double x1; // Radii
		// double x2;
		
		// x1, x2
		b2Vec2 radii;
		b2Vec2 naturalRadii;
				
		b2MassData massData;

		double youngsModulus;
		double shearModulus;
		
		double restitution;
		double friction;
		double density;
		
		double shearVelocity;
		b2Vec2 xsVelocity;
		double angularVelocity;
		// b2Vec2 velocity;
		
		double axis1Offset;
		
		double angularDamping;
		double linearDamping;
		
		// Update the vertices and normals of the fixture.
		void setVerticesNormals();
		
		b2Vec2 getReaction(int side);
		
		b2Vec2 getAxis2() {
			return b2Mul(b2Rot(theta), axis1);
		}
		
		b2Vec2 getNormal(int side) {
			b2Vec2 axis = axis1;
			if (side % 2 == 1) {
				axis = getAxis2();
			}
			if (side > 1) {
				axis = -axis;
			}
			return axis;
		}
		
		b2Vec2 getPoint(int side) {
			b2Vec2 axis = getNormal(side);
			axis *= radii(side % 2);
			return center + axis;
		}
};