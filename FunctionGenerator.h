#ifndef __FGENERATE__
#define __FGENERATE__

#include "Timer.h"

class FunctionGenerator {
	Timer timer;

public:
	FunctionGenerator() {
		timer.reset();
	}

	float makeSineWave(float magnitude, float rps) {
		return magnitude * sin(2 * M_PI * rps * timer.dt());
	}
	

};

#endif


