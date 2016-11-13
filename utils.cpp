#include <cmath>
#include "btBulletDynamicsCommon.h"
#include <iostream>
#include "Vector3.h"
using namespace std;

void centroid_triangle(int n, const double *x, const double *y, double *result) {
	double A = 0;
	for (int i = 0; i < n; ++ i)
		A += x[i] * y[(i + 1) % n] - x[(i + 1) % n] * y[i];
	
	// find the center of mass
	//A = 1.0 / (6.0 * A);
	result[0] = result[1] = result[2] = 0;
	for (int i = 0; i < n; ++ i) {
		double tmp = x[i] * y[(i + 1) % n] - x[(i + 1) % n] * y[i];
		result[0] += (x[i] + x[(i + 1) % n]) * tmp;
		result[1] += (y[i] + y[(i + 1) % n]) * tmp;
	}
	result[0] /= 3.0 * A;
	result[1] /= 3.0 * A;

	// calculate the area
	for (int i = 1; i < n - 1; ++ i) {
		double x1 = x[i] - x[0], x2 = x[i + 1] - x[0];
		double y1 = y[i] - y[0], y2 = y[i + 1] - y[0];
		result[2] += x1 * y2 - x2 * y1;
	}
	result[2] = abs(result[2] * 0.5);
}

// cone shaped centroid
Vector3 centroid(int n, const Vector3 *pts) {
	// consider each side
	double total_mass = 0;
	double result[3], x[n], y[n];
	Vector3 ret = Vector3(0, 0, 0);

	// bottom shape
	for (int i = 0; i < n - 1; ++ i) {
		x[i] = pts[i + 1].x();
		y[i] = pts[i + 1].z();
	}
	centroid_triangle(n - 1, x, y, result);
	ret += Vector3(result[0], 0, result[1]) * result[2];
	total_mass += result[2];

	// side shape
	for (int i = 0; i < n - 1; ++ i) {
		Vector3 X, Y, Z;
		Vector3 A, B;
		Vector3 c;
		A = pts[i + 1] - pts[0]; B = pts[((i + 1) % (n - 1)) + 1] - pts[0];
		X = A.normalize(); Y = B.normalize();
		Z = X.cross(Y).normalize(); Y = Z.cross(X).normalize();
		x[0] = y[0] = 0;
		x[1] = A.dot(X); y[1] = A.dot(Y);
		x[2] = B.dot(X); y[2] = B.dot(Y);

		/*
		cerr << sizeof(result[0]) << endl;
		cerr << X.dot(Y) << ' ' << Y.dot(Z) << ' ' << Z.dot(X) << endl;
		cerr << X.length() << ' ' << Y.length() << ' ' << Z.length() << endl;
		cerr << A.length() << ' ' << B.length() << endl;
		cerr << x[1] << ' ' << y[1] << ' ' << x[2] << ' ' << y[2] << endl;
		*/
		
		centroid_triangle(3, x, y, result);

		//cerr << result[0] << ' ' << result[1] << ' ' << result[2] << endl;

		c = pts[0] + X * result[0] + Y * result[1];
		ret += c * 0.5 * A.cross(B).length();
		total_mass += 0.5 * A.cross(B).length();
	}
	return ret / total_mass;
}

