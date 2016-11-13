CC = g++
CFLAGS = -I/usr/include/bullet
LIBS = -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath -lGL -lGLU -lglut
target ?= helloworld

all:
	$(CC) $(CFLAGS) $(target).cpp -o $(target) $(LIBS)
