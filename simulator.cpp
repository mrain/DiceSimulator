#include <iostream>
#include "simulator.h"
#include <cstdlib>
#include <ctime>
#include <cmath>
using namespace std;

int freq[10];

const double pi = acos(-1.);

int main() {

	const int n = 6;
	float angles[n];
	const float h = 0.3;
	int tot = 0;
	for (int i = 0; i < n - 1; ++ i)
		tot += (1 << i) + 5;
	for (int i = 0; i < n - 1; ++ i)
		angles[i] = pi * 2.0 * ((1 << i) + 5) / tot;
	

	srand(time(0));
	for (int i = 0; i < 10000; ++ i) {
		if (i % 1000 == 0) cout << i << endl;
		//cout << simulator->simulate() << endl;
		//simulator->reset();
		Simulator *simulator = new Simulator(n, angles, h);
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
