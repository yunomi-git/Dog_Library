#ifndef __ROTPOINT__
#define __ROTPOINT__

#include "math2.h"
#include "math.h"

#define ROT_ZERO Rot(0,0,0)

// A 3d rotational state
// CCW=+

struct Rot {
    float x; // roll=x
    float y; // pitch=y
    float z; // yaw=z

public:
    // Constructors
    Rot() {x = 0; y = 0; z = 0;}

    Rot(float nx, float ny, float nz) {
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
    Rot operator*(float k) {
        return Rot(k*x, k*y, k*z);
    }

    Rot operator/(float k) {
        return Rot(x/k, y/k, z/k);
    }

    void operator*=(float k) {
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