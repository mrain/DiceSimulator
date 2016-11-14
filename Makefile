CC = g++
CFLAGS = -I/usr/include/bullet
LIBS = -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath -lGL -lGLU -lglut
#target ?= graphics

all: graphics

simulator.o: simulator.cpp
	$(CC) -c $(CFLAGS) simulator.cpp

graphics.o: graphics.cpp
	$(CC) -c $(CFLAGS) graphics.cpp

graphics: simulator.o graphics.o
	$(CC) simulator.o graphics.o -o graphics $(LIBS)

clean:
	rm *.o
	rm graphics
