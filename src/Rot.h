#ifndef __ROTPOINT__
#define __ROTPOINT__

#include "math2.h"
#include "math.h"

#define ROT_ZERO Rot(0,0,0)

// A 3d rotational state
// CCW=+

struct Rot {
    double x; // roll=x
    double y; // pitch=y
    double z; // yaw=z

public:
    // Constructors
    Rot() {x = 0; y = 0; z = 0;}

    Rot(double nx, double ny, double nz) {
     	x = nx; y = ny; z = nz;
    }

    // Rot
    Rot operator+(Rot r) {
        return Rot(x+r.x, y+r.y, z+r.z);
    }

    Rot operator-(Rot r) {
        return Rot(x-r.x, y-r.y, z-r.z);
    }

    void operator+=(Rot r) {
        x += r.x;
        y += r.y;
        z += r.z;
    }

    // Constant
    Rot operator*(double k) {
        return Rot(k*x, k*y, k*z);
    }

    Rot operator/(double k) {
        return Rot(x/k, y/k, z/k);
    }

    void operator*=(double k) {
        x *= k;
        y *= k;
        z *= k;
    }

    // Conversion to boolean
    explicit operator bool() const {
        return !((x==0) && (y==0) && (z==0));
    }

    Rot operator-() {
        return (*this) * -1;
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