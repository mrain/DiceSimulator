#include "btBulletDynamicsCommon.h"
#include <cstdio>
#include <cmath>
#include <iostream>
#include "utils.cpp"
#include "Vector3.h"
#include <GL/glut.h>
using namespace std;

const double pi = acos(-1.);
const char title[] = "Dice Simulator";
float anglePyramid = 0.0;
int refreshMills = 10;

/// This is a Hello World program for running a basic Bullet physics simulation

btDiscreteDynamicsWorld *world;
btAlignedObjectArray<btCollisionShape*> collisionShapes;
btDefaultCollisionConfiguration *collisionConfiguration;
btCollisionDispatcher *dispatcher;
btBroadphaseInterface *overlappingPairCache;
btSequentialImpulseConstraintSolver* solver;

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

	//cerr << ((btConvexHullShape *)colShape)->getNumEdges() << endl;
	//cerr << ((btConvexHullShape *)colShape)->getNumPlanes() << endl;
	collisionShapes.push_back(colShape);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

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

void simulate() {
	world->stepSimulation(((btScalar)refreshMills) / 1000.0, 10);
	
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
		if (j == 1)
			printf("world pos object %d = %f,%f,%f\n",j,float(trans.getOrigin().getX()),float(trans.getOrigin().getY()),float(trans.getOrigin().getZ()));
	}
}

/* Initialize OpenGL Graphics */
void initGL() {
   glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
   glClearDepth(1.0f);                   // Set background depth to farthest
   glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
   glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
   glShadeModel(GL_SMOOTH);   // Enable smooth shading
   glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections

   initBulletWorld();
   createGround();
   createDice();
}

void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
	glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

	simulate();
	btCollisionObject *obj = world->getCollisionObjectArray()[1];
	btRigidBody *body = btRigidBody::upcast(obj);
	btTransform trans;
	if (body && body->getMotionState()) {
		body->getMotionState()->getWorldTransform(trans);
	} else {
		trans = obj->getWorldTransform();
	}


	glLoadIdentity();                  // Reset the model-view matrix
	glTranslatef(-1.5f, 0.0f, -20.0f);  // Move left and into the screen
	//glRotatef(anglePyramid, 1.0f, 1.0f, 0.0f);  // Rotate about the (1,1,0)-axis [NEW]

	glBegin(GL_TRIANGLES);
	//btConvexHullShape *shape = (btConvexHullShape *)collisionShapes[1];
	btConvexHullShape *shape = (btConvexHullShape *)obj->getCollisionShape();
	glColor3f(0, 0, 1);
	for (int i = 0; i < shape->getNumPoints(); ++ i) {
		btVector3 a, b, c;
		shape->getVertex(i, a);
		a = trans * a;
		for (int j = 0; j < shape->getNumEdges(); ++ j) {
			shape->getEdge(j, b, c);
			b = trans * b;
			c = trans * c;
			glVertex3f(a.x(), a.y(), a.z());
			glVertex3f(b.x(), b.y(), b.z());
			glVertex3f(c.x(), c.y(), c.z());
		}
	}
	glEnd();

/*
	glBegin(GL_POLYGON);

	for (int i = 0; i < shape->getNumPoints(); ++ i) {
		btVector3 point = shape->getUnscaledPoints()[i];
		glVertex3f(point.x(), point.y(), point.z());
	}

	glEnd();*/
	/*
	glBegin(GL_TRIANGLES);           // Begin drawing the pyramid with 4 triangles
	  // Front
		glColor3f(1.0f, 0.0f, 0.0f);     // Red
		glVertex3f( 0.0f, 1.0f, 0.0f);
		glColor3f(0.0f, 1.0f, 0.0f);     // Green
		glVertex3f(-1.0f, -1.0f, 1.0f);
		glColor3f(0.0f, 0.0f, 1.0f);     // Blue
		glVertex3f(1.0f, -1.0f, 1.0f);

		// Right
		glColor3f(1.0f, 0.0f, 0.0f);     // Red
		glVertex3f(0.0f, 1.0f, 0.0f);
		glColor3f(0.0f, 0.0f, 1.0f);     // Blue
		glVertex3f(1.0f, -1.0f, 1.0f);
		glColor3f(0.0f, 1.0f, 0.0f);     // Green
		glVertex3f(1.0f, -1.0f, -1.0f);

		// Back
		glColor3f(1.0f, 0.0f, 0.0f);     // Red
		glVertex3f(0.0f, 1.0f, 0.0f);
		glColor3f(0.0f, 1.0f, 0.0f);     // Green
		glVertex3f(1.0f, -1.0f, -1.0f);
		glColor3f(0.0f, 0.0f, 1.0f);     // Blue
		glVertex3f(-1.0f, -1.0f, -1.0f);

		// Left
		glColor3f(1.0f,0.0f,0.0f);       // Red
		glVertex3f( 0.0f, 1.0f, 0.0f);
		glColor3f(0.0f,0.0f,1.0f);       // Blue
		glVertex3f(-1.0f,-1.0f,-1.0f);
		glColor3f(0.0f,1.0f,0.0f);       // Green
		glVertex3f(-1.0f,-1.0f, 1.0f);
	glEnd();   // Done drawing the pyramid
		*/

	glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
	// Update the rotational angle after each refresh [NEW]
	anglePyramid += 0.2f;
}

void reshape(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
   // Compute aspect ratio of the new window
   if (height == 0) height = 1;                // To prevent divide by 0
   GLfloat aspect = (GLfloat)width / (GLfloat)height;
 
   // Set the viewport to cover the new window
   glViewport(0, 0, width, height);
 
   // Set the aspect ratio of the clipping volume to match the viewport
   glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
   glLoadIdentity();             // Reset
   // Enable perspective projection with fovy, aspect, zNear and zFar
   gluPerspective(45.0f, aspect, 0.1f, 100.0f);
}

void timer(int value) {
	glutPostRedisplay();
	glutTimerFunc(refreshMills, timer, 0);
}

int main(int argc, char** argv) {
	glutInit(&argc, argv);            // Initialize GLUT
	glutInitDisplayMode(GLUT_DOUBLE); // Enable double buffered mode
	glutInitWindowSize(640, 480);   // Set the window's initial width & height
	glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
	glutCreateWindow(title);          // Create window with the given title
	glutDisplayFunc(display);       // Register callback handler for window re-paint event
	glutReshapeFunc(reshape);       // Register callback handler for window re-size event
	initGL();                       // Our own OpenGL initialization
	glutTimerFunc(0, timer, 0);     // First timer call immediately [NEW]
	glutMainLoop();                 // Enter the infinite event-processing loop

//	initBulletWorld();
//	createGround();
//	createDice();

	///-----stepsimulation_start-----
//	for (int i = 0; i < 200; ++ i)
//		simulate();

	///-----stepsimulation_end-----
//	destroyBulletWorld();
}

