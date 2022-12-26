#ifndef __MOTIONCONTROLLER__
#define __MOTIONCONTROLLER__

struct WholeMotionCommand {
	float vel_x, vel_y, vel_z;
	float rvel_x, rvel_y, rvel_z;
};

class MotionController {
private:
	RobotDog *dog_r;
	WholeMotionCommand whole_motion_command;

	CreepGaitCoodinator gait_coordinator;
	CreepGaitStateMachine gait_state_machine(&gait_coordinator);
public:
	MotionController(RobotDog *ndog) {
		dog_r = ndog;
		gait_coordinator = CreepGaitCoodinator(dog_r);

	}


	void recieveMotionCommand(float vx, float vy, float vz, float rvx, float rvy, float rvz) {
		whole_motion_command.vel_x = vx;
		whole_motion_command.vel_y = vy;
		whole_motion_command.vel_z = vz;
		whole_motion_command.rvel_x = rvx;
		whole_motion_command.rvel_y = rvy;
		whole_motion_command.rvel_z = rvz;
	}

	void decideMotionToTake() {
		if (dogIsWithinSafeRegion()) {
			translateBody();
		} else {
			StepMotionCommand step_motion_command;
			step_motion_command.x_motion;
			step_motion_command.y_motion;
			step_motion_command.yaw_motion;
			takeAStep(step_motion_command);
		}
	}




};


#endif