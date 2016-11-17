CC = g++
CFLAGS = -I/usr/include/bullet -O2
LIBS = -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath -lGL -lGLU -lglut
#target ?= graphics

all: graphics simulator

graphics.o: simulator.h graphics.cpp
	$(CC) -c $(CFLAGS) graphics.cpp

graphics: graphics.o
	$(CC) graphics.o -o graphics $(LIBS)

simulator: simulator.h simulator.cpp
	$(CC) $(CFLAGS) simulator.cpp -o simulator $(LIBS)

clean:
	rm *.o
	rm graphics
	rm simulator
