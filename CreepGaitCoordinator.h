#ifndef __CreepGaitCoordinator__
#define __CreepGaitCoordinator__

// The coordinator computes foot and body positions for the gait state machine to reach in a given foot step.
// It does not decide when a foot step should occurs

#include "CreepTimingParameters.h"
#include "Dog.h"
#include "DesiredCreepMotionCommand.h"


struct footCommand {
	int foot_to_move = 0;
	bool overrideFootChoice = false;
	Point desired_foot_anchor_from_original_centroid_oC;

};

struct COMCommand {
	Point desired_COM_from_original_centroid_oC;
	float motion_time;


};


#define BODY_DEFAULT_HEIGHT 110


class CreepGaitCoordinator {
public:
	CreepGaitCoordinator() {

	}

	CreepGaitCoordinator(RobotDog *ndog_r) {
		dog_r = ndog_r;
		dog_r->printLegs();
	}

	virtual footCommand getNextFootCommand() = 0;

	virtual COMCommand getNextCOMCommand() = 0;

	Point getDefaultBodyPosition() {
		return Point(0, 0, BODY_DEFAULT_HEIGHT);
	}

	CreepTimingParameters getTimingParameters() {
		return timingParameters;
	}

    RobotDog *getDogReference() {
    	return dog_r;
    }

    void receiveDesiredMotion(DesiredCreepMotionCommand ndesired_motion) {
    	desired_motion = ndesired_motion;
    }



private:
protected:
	RobotDog *dog_r;
	CreepTimingParameters timingParameters;
	DesiredCreepMotionCommand desired_motion;


};



#endif