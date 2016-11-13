#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>
#include "btBulletDynamicsCommon.h"

class Vector3 {
private:
	double X, Y, Z;

public:
	Vector3() {}
	Vector3(double x, double y, double z): X(x), Y(y), Z(z) {}
	Vector3(const btVector3 &v): X(v.x()), Y(v.y()), Z(v.z()) {}
	double x() const { return X; }
	double y() const { return Y; }
	double z() const { return Z; }

	void setX(double x) { X = x; }
	void setY(double y) { Y = y; }
	void setZ(double z) { Z = z; }

	Vector3 & operator += (const Vector3 &v) {
		X += v.x(); Y += v.y(); Z += v.z();
		return *this;
	}
	Vector3 & operator -= (const Vector3 &v) {
		X -= v.x(); Y -= v.y(); Z -= v.z();
		return *this;
	}
	Vector3 & operator *= (const double &v) {
		X *= v; Y *= v; Z *= v;
		return *this;
	}
	Vector3 & operator /= (const double &v) {
		X /= v; Y /= v; Z /= v;
		return *this;
	}
	Vector3 & operator *= (const Vector3 &v) {
		X *= v.x(); Y *= v.y(); Z *= v.z();
		return *this;
	}

	Vector3 operator +(const Vector3 &v) const {
		return Vector3(X + v.x(), Y + v.y(), Z + v.z());
	}
	Vector3 operator -(const Vector3 &v) const {
		return Vector3(X - v.x(), Y - v.y(), Z - v.z());
	}
	Vector3 operator *(const double &v) const {
		return Vector3(X * v, Y * v, Z * v);
	}
	Vector3 operator /(const double &v) const {
		return Vector3(X / v, Y / v, Z / v);
	}
	Vector3 operator *(const Vector3 &v) const {
		return Vector3(X * v.x(), Y * v.y(), Z * v.z());
	}

	double dot(const Vector3 &v) const{
		return X * v.x() + Y * v.y() + Z * v.z();
	}
	double length() const{
		return sqrt(dot(*this));
	}
	Vector3 cross(const Vector3 &v) const {
		return Vector3(Y * v.z() - Z * v.y(),
					   Z * v.x() - X * v.z(),
					   X * v.y() - Y * v.x());
	}
	Vector3 normalize() const {
		if (length() < 1e-10) return *this;
		else return (*this) / length();
	}

	btVector3 toBullet() const{
		return btVector3(X, Y, Z);
	}
};

#endif
