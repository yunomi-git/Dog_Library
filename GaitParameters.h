#ifndef __GAITPARAMETERS__
#define __GAITPARAMETERS__

struct trotFootGroup {
	int foot1;
	int foot2;
};

struct trotTimingParameters {
	float prepare_time;
	float prepare_overlap_factor;
	float lift_time;
	float lift_overlap_factor;
	float plant_time;
	float plant_overlap_factor;
	float return_time;
	float return_overlap_factor;
};

trotTimingParameters FAST_TROT_PARAMETERS {	0.35, 	0.6, 	// prepare
											0.1, 	0, 		// lift
											0.15, 	0, 		// plant
											0.25, 	0};		// return

trotTimingParameters SLOW_TROT_PARAMETERS {	.5, 	0.0, 	// prepare
											.1, 	0, 		// lift
											.1, 	0, 		// plant
											.5, 	0};		// return


const float ROTATION_LEG_INPUT_MULTIPLIER = 3;

const float BODY_DEFAULT_HEIGHT = 110;
const float LIFT_HEIGHT = 30;

#define CALL_MEMBER_FN(object,ptrToMember)  ((object)->*(ptrToMember))

#endif
