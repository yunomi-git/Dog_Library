#ifndef __20KGSERVO__
#define __20KGSERVO__

#include "RServo.h"


// This stepper is meant to be used in Arduino's loop as a non-blocking stepper. Operate must be called each loop.
// This stepper represents that used by the carousel (ie it uses that stepper controller)

#define min_ang -90
#define max_ang 90
#define min_sig 850
#define max_sig 2150

class kg20servo : public RServo{

public:
	kg20servo(double nspeed = 1) : RServo(min_ang, max_ang, min_sig, max_sig, nspeed) {
	}
};

#endif