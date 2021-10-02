#ifndef __CreepGaitCoordinator__
#define __CreepGaitCoordinator__

// The coordinator computes foot and body positions for the gait state machine to reach in a given foot step.
// It does not decide when a foot step should occurs

#include "CreepTimingParameters.h"

class CreepGaitCoordinator {
public:
	CreepGaitCoordinator(Dog *dog_r) {}

	virtual footCommand getNextFootCommand() = 0;

	virtual COMCommand getNextCOMCommand() = 0;

	CreepTimingParameters getTimingParameters() {
		return timingParameters;
	}

    Dog *getDogReference() {
    	return dog_r;
    }

private:
	Dog *dog_r;
protected:
	CreepTimingParameters timingParameters;


}

struct footCommand() {
	int foot_to_move = 0;
	bool overrideFootChoice = false;
	Point foot_anchor_before_motion_oC;

}

struct COMCommand() {
	Point desired_COM_location_before_motion_oC; // relative to what? original centroid?
	float motion_time;


}


#endif