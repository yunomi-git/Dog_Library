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
     Rot() {x = 0; y = 0; z = 0;}

     Rot(double nx, double ny, double nz) {
     	x = nx; y = ny; z = nz;
     }

     Rot operator+(Rot p) {
        return Rot(x+p.x, y+p.y, z+p.z);
     }

     Rot operator-(Rot p) {
        return Rot(x-p.x, y-p.y, z-p.z);
     }

     Rot operator*(double k) {
        return Rot(k*x, k*y, k*z);
     }

     void operator*=(double k) {
        x *= k;
        y *= k;
        z *= k;
     }
};

#endif