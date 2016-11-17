#ifndef NAMESPACE_BULLET
#define NAMESPACE_BULLET
#include "btBulletDynamicsCommon.h"
#include "Vector3.h"
#include <cstdlib>
#include <algorithm>
#include <ctime>
// debug
#include <iostream>

class Simulator {
private:
	const double pi = acos(-1.);
	btDiscreteDynamicsWorld *world;
	//btAlignedObjectArray<btCollisionShape*> collisionShapes;
	btDefaultCollisionConfiguration *collisionConfiguration;
	btCollisionDispatcher *dispatcher;
	btBroadphaseInterface *overlappingPairCache;
	btSequentialImpulseConstraintSolver* solver;
	btCollisionShape *groundShape;
	btConvexHullShape *diceShape;

	void centroid_triangle(int n, const double *x, const double *y, double *result) {
		double A = 0;
		for (int i = 0; i < n; ++ i)
			A += x[i] * y[(i + 1) % n] - x[(i + 1) % n] * y[i];
		
		// find the center of mass
		//A = 1.0 / (6.0 * A);
		result[0] = result[1] = result[2] = 0;
		for (int i = 0; i < n; ++ i) {
			double tmp = x[i] * y[(i + 1) % n] - x[(i + 1) % n] * y[i];
			result[0] += (x[i] + x[(i + 1) % n]) * tmp;
			result[1] += (y[i] + y[(i + 1) % n]) * tmp;
		}
		result[0] /= 3.0 * A;
		result[1] /= 3.0 * A;

		// calculate the area
		for (int i = 1; i < n - 1; ++ i) {
			double x1 = x[i] - x[0], x2 = x[i + 1] - x[0];
			double y1 = y[i] - y[0], y2 = y[i + 1] - y[0];
			result[2] += x1 * y2 - x2 * y1;
		}
		result[2] = abs(result[2] * 0.5);
	}

	// cone shaped centroid
	Vector3 centroid(int n, const Vector3 *pts) {
		// consider each side
		double total_mass = 0;
		double result[3], x[n], y[n];
		Vector3 ret = Vector3(0, 0, 0);

		// bottom shape
		for (int i = 0; i < n - 1; ++ i) {
			x[i] = pts[i + 1].x();
			y[i] = pts[i + 1].z();
		}
		centroid_triangle(n - 1, x, y, result);
		ret += Vector3(result[0], 0, result[1]) * result[2];
		total_mass += result[2];

		// side shape
		for (int i = 0; i < n - 1; ++ i) {
			Vector3 X, Y, Z;
			Vector3 A, B;
			Vector3 c;
			A = pts[i + 1] - pts[0]; B = pts[((i + 1) % (n - 1)) + 1] - pts[0];
			X = A.normalize(); Y = B.normalize();
			Z = X.cross(Y).normalize(); Y = Z.cross(X).normalize();
			x[0] = y[0] = 0;
			x[1] = A.dot(X); y[1] = A.dot(Y);
			x[2] = B.dot(X); y[2] = B.dot(Y);

			centroid_triangle(3, x, y, result);

			//cerr << result[0] << ' ' << result[1] << ' ' << result[2] << endl;

			c = pts[0] + X * result[0] + Y * result[1];
			ret += c * 0.5 * A.cross(B).length();
			total_mass += 0.5 * A.cross(B).length();
		}
		return ret / total_mass;
	}

	double randomDouble(double Min, double Max) {
		double f = (double)rand() / RAND_MAX;
		return Min + f * (Max - Min);
	}

	void initBulletWorld() {
		collisionConfiguration = new btDefaultCollisionConfiguration();

		///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		dispatcher = new btCollisionDispatcher(collisionConfiguration);

		///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
		overlappingPairCache = new btDbvtBroadphase();

		///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
		solver = new btSequentialImpulseConstraintSolver();

		world = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

		world->setGravity(btVector3(0,-10,0));
		///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	}

	void createGround() {
		//the ground is a cube of side 100 at position y = -56.
		//the sphere will hit it at y = -6, with center at -5

		groundShape = new btBoxShape(btVector3(btScalar(500.),btScalar(500.),btScalar(500.)));

		//collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-506,0));

		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		world->addRigidBody(body);
	}

	void createDice() {
		Vector3 points[6];

		points[0] = Vector3(0,1,0);
		double tmp = pi * 2.0 / 5.0;
		for (int i = 1; i < 6; ++ i)
			points[i] = Vector3(cos(tmp * i), 0, sin(tmp * i));

		//for (int i = 0; i < 6; ++ i) 
	//		cerr << points[i].x() << ' ' << points[i].y() << ' ' << points[i].z() << endl;
		Vector3 c = centroid(6, points);
	//	cerr << c.x() << ' ' << c.y() << ' ' << c.z() << endl;

		btVector3 pts[6];
		for (int i = 0; i < 6; ++ i)
			pts[i] = points[i].toBullet();

		diceShape = new btConvexHullShape((btScalar *)&pts, 6);

		//collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();
		btScalar rx = randomDouble(-pi, pi);
		btScalar ry = randomDouble(-pi, pi);
		btScalar rz = randomDouble(-pi, pi);
		startTransform.setRotation(btQuaternion(btVector3(1, 0, 0), rx) * btQuaternion(btVector3(0, 1, 0), ry) * btQuaternion(btVector3(0, 0, 1), rz));

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic) {
			diceShape->calculateLocalInertia(mass,localInertia);
	//		cerr << localInertia.x() << ' ' << localInertia.y() << ' ' << localInertia.z() << endl;
		}

		//Move to a certain height
		startTransform.setOrigin(btVector3(0,10,0));

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,(btCollisionShape *)diceShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		// Apply an initial velocity
		btScalar velocityScale = 10;
		rx = randomDouble(-pi, pi);
		ry = randomDouble(-pi, pi);
		rz = randomDouble(-pi, pi);
		body->setLinearVelocity(
				btVector3(velocityScale, 0, 0)
				.rotate(btVector3(0, 0, 1), rz)
				.rotate(btVector3(0, 1, 0), ry)
				.rotate(btVector3(1, 0, 0), rx)
			);
		rx = randomDouble(-pi, pi);
		ry = randomDouble(-pi, pi);
		rz = randomDouble(-pi, pi);
		body->setAngularVelocity(
				btVector3(velocityScale, 0, 0)
				.rotate(btVector3(0, 0, 1), rz)
				.rotate(btVector3(0, 1, 0), ry)
				.rotate(btVector3(1, 0, 0), rx)
			);


		world->addRigidBody(body);

	}

	void destroyBulletWorld() {
		for (int i = world->getNumCollisionObjects() - 1; i >= 0 ; --i) {
			btCollisionObject* obj = world->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
				delete body->getMotionState();

			world->removeCollisionObject( obj );
			delete obj;
		}

		//delete collision shapes
		/*for (int j = 0; j < collisionShapes.size(); ++ j) {
			btCollisionShape* shape = collisionShapes[j];
			collisionShapes[j] = 0;
			delete shape;
		}*/
		delete groundShape;
		delete diceShape;

		delete world;

		delete solver;

		//delete broadphase
		delete overlappingPairCache;

		//delete dispatcher
		delete dispatcher;

		delete collisionConfiguration;

		//collisionShapes.clear();

	}

public:
	void stepSimulate(btScalar secs) {
		world->stepSimulation(secs, 10);
		
		//print positions of all objects
		/*for (int j=world->getNumCollisionObjects()-1; j>=0 ;j--) {
			btCollisionObject* obj = world->getCollisionObjectArray()[j];
			btRigidBody* body = btRigidBody::upcast(obj);
			btTransform trans;
			if (body && body->getMotionState()) {
				body->getMotionState()->getWorldTransform(trans);
			} else {
				trans = obj->getWorldTransform();
			}
			if (j == 1) {
				//printf("world pos object %d = %f,%f,%f\n",j,float(trans.getOrigin().getX()),float(trans.getOrigin().getY()),float(trans.getOrigin().getZ()));
				btConvexHullShape *shape = (btConvexHullShape *) collisionShapes[j];
				for (int i = 0; i < shape->getNumVertices(); ++ i) {
					btVector3 a;
					shape->getVertex(i, a);
					a = trans * a;
					//cerr << "(" << a.x() << ',' << a.y() << ',' << a.z() << ") ";
				}
				//cerr << endl;
			}
		}*/
	}

	// roll the dice once, return the face-down
	int simulate() {
		// get Dice
		btCollisionObject *obj = world->getCollisionObjectArray()[1];
		btRigidBody *body = btRigidBody::upcast(obj);
		btTransform trans;

		int cnt = 0;
		while (true) {
			stepSimulate(0.01);
			if (body->getLinearVelocity().length() == 0 && body->getAngularVelocity().length() == 0) {
				++ cnt;
				if (cnt > 10) {
					if (body->getMotionState())
						body->getMotionState()->getWorldTransform(trans);
					else
						trans = obj->getWorldTransform();
					break;
				}
			} else {
				cnt = 0;
			}
		}
		/*
		double height = 100; 
		int count = 0;
		while (true) {
			// step size 10ms
			stepSimulate(0.01);

			if (body->getMotionState()) {
				body->getMotionState()->getWorldTransform(trans);
			} else {
				trans = obj->getWorldTransform();
			}

			//std::cerr << body->getLinearVelocity().length() << ' ' << body->getAngularVelocity().length() << std::endl;

			double now = trans.getOrigin().y();
			if (abs(now - height) < 1e-6) {
				++ count;
				if (count > 100) break;
			} else count = 0;
			height = now;
		}*/

		// Check every face
		int n = diceShape->getNumVertices();
		btScalar h[n];
		btScalar ground = 100;
		for (int i = 0; i < n; ++ i) {
			btVector3 pts;
			diceShape->getVertex(i, pts);
			pts = trans * pts;
			h[i] = pts.y();
			ground = std::min(ground, h[i]);
			//std::cerr << h[i] << ' ';
		}
		//std::cerr << std::endl;

		//bottom on the ground
		if (h[0] - ground > 1e-2) return 0;

		for (int i = 1; i < n; ++ i) {
			if (h[i] - ground <= 1e-2 && h[(i % (n - 1)) + 1] - ground <= 1e-2) {
				//std::cerr << "!" << i << std::endl;
				return i;
			}
		}

		for (int i = 0; i < n; ++ i)
			std::cerr << h[i] << ' ';
		std::cerr << std::endl;
		return -1;
	}

	btConvexHullShape currentDiceShape() {
		btCollisionObject *obj = world->getCollisionObjectArray()[1];
		btRigidBody *body = btRigidBody::upcast(obj);
		btTransform trans;
		if (body && body->getMotionState()) {
			body->getMotionState()->getWorldTransform(trans);
		} else {
			trans = obj->getWorldTransform();
		}

		btVector3 pts[diceShape->getNumVertices()];
		for (int i = 0; i < diceShape->getNumVertices(); ++ i) {
			diceShape->getVertex(i, pts[i]);
			pts[i] = trans * pts[i];
		}

		btConvexHullShape ret((btScalar *)pts, diceShape->getNumVertices());
		return ret;
	}

	Simulator() {
		initBulletWorld();
		createGround();
		createDice();
	}

	~Simulator() {
		destroyBulletWorld();
	}

};
#endif


