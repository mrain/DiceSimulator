#include <iostream>
#include "simulator.h"
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstring>
using namespace std;

const int rounds = 10000;

const double pi = acos(-1.);

void simulation(int n, const float *angles, const float h) {
	int freq[n];
	memset(freq, 0, sizeof(freq));
	for (int i = 0; i < rounds; ++ i) {
		if (i % 1000 == 0) cout << i << endl;
		//cout << simulator->simulate() << endl;
		//simulator->reset();
		Simulator *simulator = new Simulator(n, angles, h);
		int ret = simulator->simulate();
		if (ret < 0) cerr << "!" << endl;
		freq[ret] ++;
		delete simulator;
	}

	cout << "----------------shape---------------" << endl;
	Simulator::printDiceShape(n, angles, h);

	cout << "------------frequency---------------" << endl;
	for (int i = 0; i < n; ++ i) {
		cout << i << ' ' << freq[i] << endl;
	}

	vector<double> p;
	for (int i = 0; i < (1 << n); ++ i) {
		double t = 0;
		for (int k = 0; k < n; ++ k) {
			if (i & (1 << k))
				t += ((double)freq[k]) / rounds;
		}
		p.push_back(t);
	}
	sort(p.begin(), p.end());

	double gap = p[1] - p[0];
	for (int i = 1; i < p.size() - 1; ++ i)
		gap = max(gap, p[i + 1] - p[i]);
	cout << "Estimated gap: " << gap << endl;
}

int main() {

	srand(time(0));

	const int n = 6;
	int weight[n];
	float angles[n];
	float h = 0.3;
	int tot = 0;
	for (int i = 0; i < n - 1; ++ i)
		tot += (1 << i) + (n - i);
	for (int i = 0; i < n - 1; ++ i)
		angles[i] = pi * 2.0 * ((1 << i) + (n - i)) / tot;

	simulation(n, angles, h);

	while (true) {
		tot = 0;
		for (int i = 0; i < n - 1; ++ i) {
			cin >> weight[i];
			tot += weight[i];
		}
		cin >> h;
		for (int i = 0; i < n; ++ i) {
			angles[i] = pi * 2.0 * weight[i] / tot;
		}
		simulation(n, angles, h);
	}
	return 0;
}
