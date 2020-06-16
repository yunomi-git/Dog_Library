#ifndef __POINT__
#define __POINT__

#include "math2.h"
#include "math.h"
#include "Rot.h"

#define POINT_ZERO Point(0,0,0)

// A point in 3d cartesian space

struct Point {
    double x;
    double y;
    double z;

public:
    // Constructors
    Point() {x = 0; y = 0; z = 0;}

    Point(double nx, double ny, double nz) {
     	x = nx; y = ny; z = nz;
    }

    // Point-point
    Point operator+(Point p) {
        return Point(x+p.x, y+p.y, z+p.z);
    }

    Point operator-(Point p) {
        return Point(x-p.x, y-p.y, z-p.z);
    }

    void operator+=(Point p) {
        x += p.x;
        y += p.y;
        z += p.z;
    }

    void operator-=(Point p) {
        x -= p.x;
        y -= p.y;
        z -= p.z;
    }

    bool operator==(Point p) {
       return ((x==p.x) && (y==p.y) && (z==p.z));
    }

    bool operator!=(Point p) {
       return !(p==*this);
    }

    // Rot
    // r a rotational state in degrees
    Point operator*(Rot r) {
        return rotate(r.x, r.y, r.z);
    }

    // r a rotational state in degrees
    Point operator/(Rot r) {
        return inv_rotate(r.x, r.y, r.z);
    }

    void operator*=(Rot r) {
        Point p = rotate(r);
        x = p.x;
        y = p.y;
        z = p.z;
    }

    void operator/=(Rot r) {
        Point p = inv_rotate(r);
        x = p.x;
        y = p.y;
        z = p.z;
    }

    // Constant
    Point operator*(double k) {
        return Point(k*x, k*y, k*z);
    }

    Point operator/(double k) {
        return Point(x/k, y/k, z/k);
    }

    void operator*=(double k) {
        x *= k;
        y *= k;
        z *= k;
    }

    void operator/=(double k) {
        x /= k;
        y /= k;
        z /= k;
    }

    // Etc Operator
    double norm() {
        return sqrt(x*x+y*y+z*z);
    }

    Point unit() {
        return (*this)/(*this).norm();
    }

    Point operator-() {
        return (*this) * -1;
    }

// Helper Functions
    // Conversion to boolean
    explicit operator bool() const {
        return !((x==0) && (y==0) && (z==0));
    }

    // Rotate point CCW about x axis by <angle> degrees
    Point rotX(double angle) {
    	angle = angle DEG; // to radians
    	return Point(							   x,
    				 cos(angle) * y - sin(angle) * z,
    				 sin(angle) * y + cos(angle) * z);
    }

    // Rotate point CCW about y axis by <angle> degrees
    Point rotY(double angle) {
    	angle = angle DEG; // to radians
    	return Point(cos(angle) * x + sin(angle) * z,
    				 							   y,
    				-sin(angle) * x + cos(angle) * z);
    }

     // Angle in deg
    Point rotZ(double angle) {
     	angle = angle DEG; // to radians
     	return Point(cos(angle) * x - sin(angle) * y,
     				 sin(angle) * x + cos(angle) * y,
     				 							   z);
    }

    // Rotates the point about origin by given angles in degrees in order of:
    // in GROUND/ABSOLUTE frame, 		x axis -> y axis -> z axis or,
    // in BODY/MOVING frame, 			z axis -> y axis -> x axis.
    // Note that x=roll, y=pitch, z=yaw and CCW is +.
    Point rotate(double roll, double pitch, double yaw) {
    	return rotX(roll).rotY(pitch).rotZ(yaw);
    }

    Point rotate(Rot r) {
    	return rotate(r.x, r.y, r.z);
    }

    // Reverses the rotation from r->0
    // ie. Starting in orientation (roll, pitch, yaw), orient to (0,0,0)
    Point inv_rotate(double roll, double pitch, double yaw) {
        return rotZ(-yaw).rotY(-pitch).rotX(-roll);
    }

    Point inv_rotate(Rot r) {
        return inv_rotate(r.x, r.y, r.z);
    }

    void print() {
        Serial.print("(X: ");   Serial.print(x);
        Serial.print(", Y:"); Serial.print(y);
        Serial.print(", Z:"); Serial.print(z);
        Serial.print(")");
        Serial.println();
    }
};

#endif