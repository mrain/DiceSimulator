#ifndef NAMESPACE_BULLET
#define NAMESPACE_BULLET
#include "btBulletDynamicsCommon.h"
#include "Vector3.h"

namespace bullet {

	extern btDiscreteDynamicsWorld *world;
	extern btAlignedObjectArray<btCollisionShape*> collisionShapes;
	extern btDefaultCollisionConfiguration *collisionConfiguration;
	extern btCollisionDispatcher *dispatcher;
	extern btBroadphaseInterface *overlappingPairCache;
	extern btSequentialImpulseConstraintSolver* solver;

	void initBulletWorld();
	void createGround();
	void createDice();
	void destroyBulletWorld();
	void stepSimulate(btScalar secs);
};
#endif


