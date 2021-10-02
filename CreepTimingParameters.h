#ifndef __CREEPTIMINGPARAMETER__
#define __CREEPTIMINGPARAMETER__

struct CreepTimingParameters {
	float prepare_motion_time;
	float prepare_state_time;
	float lift_motion_time;
	float lift_state_time;
	float plant_motion_time;
	float plant_state_time;
	float return_motion_time;
	float return_state_time;

	float getTotalStepTime() {
		return prepare_state_time + lift_state_time + plant_motion_time;
	}

	float getTotalFootSwingTime() {
		return lift_state_time + plant_motion_time;
	}
};


#endif
