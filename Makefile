CC = g++
CFLAGS = -I/usr/include/bullet
LIBS = -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath -lGL -lGLU -lglut
#target ?= graphics

all: graphics

graphics.o: graphics.cpp
	$(CC) -c $(CFLAGS) graphics.cpp

graphics: graphics.o
	$(CC) graphics.o -o graphics $(LIBS)

clean:
	rm *.o
	rm graphics
