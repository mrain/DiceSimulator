#include <iostream>
#include "simulator.h"
#include <cstdlib>
#include <ctime>
using namespace std;

int freq[10];

int main() {
	srand(time(0));
	for (int i = 0; i < 10000; ++ i) {
		if (i % 1000 == 0) cout << i << endl;
		Simulator *simulator = new Simulator();
		//cout << simulator->simulate() << endl;
		int ret = simulator->simulate();
		if (ret < 0) cerr << "!" << endl;
		freq[ret] ++;
		delete simulator;
	}
	for (int i = 0; i < 6; ++ i) {
		cout << i << ' ' << freq[i] << endl;
	}
	return 0;
}
