//#include "btBulletDynamicsCommon.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cmath>
#include <iostream>
//#include "utils.cpp"
//#include "Vector3.h"
#include <GL/glut.h>
#include "simulator.h"
using namespace std;

const double pi = acos(-1.);
const char title[] = "Dice Simulator";
float anglePyramid = 0.0;
int refreshMills = 10;
double multiplier = 2;

Simulator *simulator;

/* Initialize OpenGL Graphics */
void initGL() {
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
	glClearDepth(1.0f);                   // Set background depth to farthest
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections

	simulator = new Simulator();
   /*initBulletWorld();
   createGround();
   createDice();*/
}

void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
	glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

	simulator -> stepSimulate((btScalar)refreshMills / 1000.0 * multiplier);

	
	glLoadIdentity();                  // Reset the model-view matrix
	glTranslatef(-1.5f, 0.0f, -20.0f);  // Move left and into the screen
	//glRotatef(anglePyramid, 1.0f, 1.0f, 0.0f);  // Rotate about the (1,1,0)-axis [NEW]

	glBegin(GL_TRIANGLES);
	//btConvexHullShape *shape = (btConvexHullShape *)collisionShapes[1];

	//btConvexHullShape *shape = (btConvexHullShape *)obj->getCollisionShape();
	btConvexHullShape shape = simulator->currentDiceShape();
	glColor3f(0, 0, 1);
	for (int i = 0; i < shape.getNumPoints(); ++ i) {
		btVector3 a, b, c;
		shape.getVertex(i, a);
		//a = trans * a;
		for (int j = 0; j < shape.getNumEdges(); ++ j) {
			shape.getEdge(j, b, c);
			//b = trans * b;
			//c = trans * c;
			glVertex3f(a.x(), a.y(), a.z());
			glVertex3f(b.x(), b.y(), b.z());
			glVertex3f(c.x(), c.y(), c.z());
		}
	}
	glEnd();
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
	srand(time(0));
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
//		stepSimulate();

	///-----stepsimulation_end-----
//	destroyBulletWorld();
}

