#include "simulator.h"
#include "Vector3.h"
#include <cstdlib>
#include <ctime>

namespace bullet {

	const double pi = acos(-1.);

	btDiscreteDynamicsWorld *world;
	btAlignedObjectArray<btCollisionShape*> collisionShapes;
	btDefaultCollisionConfiguration *collisionConfiguration;
	btCollisionDispatcher *dispatcher;
	btBroadphaseInterface *overlappingPairCache;
	btSequentialImpulseConstraintSolver* solver;

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

			/*
			cerr << sizeof(result[0]) << endl;
			cerr << X.dot(Y) << ' ' << Y.dot(Z) << ' ' << Z.dot(X) << endl;
			cerr << X.length() << ' ' << Y.length() << ' ' << Z.length() << endl;
			cerr << A.length() << ' ' << B.length() << endl;
			cerr << x[1] << ' ' << y[1] << ' ' << x[2] << ' ' << y[2] << endl;
			*/
			
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

		srand(time(0));
	}

	void createGround() {
		//the ground is a cube of side 100 at position y = -56.
		//the sphere will hit it at y = -6, with center at -5

		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));

		collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-56,0));

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

		btConvexHullShape* colShape = new btConvexHullShape((btScalar *)&pts, 6);

		collisionShapes.push_back(colShape);

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
			colShape->calculateLocalInertia(mass,localInertia);
	//		cerr << localInertia.x() << ' ' << localInertia.y() << ' ' << localInertia.z() << endl;
		}

		startTransform.setOrigin(btVector3(0,10,0));

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,(btCollisionShape *)colShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

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
		for (int j = 0; j < collisionShapes.size(); ++ j) {
			btCollisionShape* shape = collisionShapes[j];
			collisionShapes[j] = 0;
			delete shape;
		}
		delete world;

		delete solver;

		//delete broadphase
		delete overlappingPairCache;

		//delete dispatcher
		delete dispatcher;

		delete collisionConfiguration;

		collisionShapes.clear();

	}

	void stepSimulate(btScalar secs) {
		world->stepSimulation(secs, 10);
		
		//print positions of all objects
		for (int j=world->getNumCollisionObjects()-1; j>=0 ;j--) {
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
				for (int i = 0; i < shape->getNumPoints(); ++ i) {
					btVector3 a;
					shape->getVertex(i, a);
					a = trans * a;
					//cerr << "(" << a.x() << ',' << a.y() << ',' << a.z() << ") ";
				}
				//cerr << endl;
			}
		}
	}


};
