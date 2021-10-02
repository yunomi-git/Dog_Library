#ifndef __CREEPCOMMANDERCONTCOM__
#define __CREEPCOMMANDERCONTCOM__

creepTimingParameters FAST_CREEP_PARAMETERS {0.35,   0,    // prepare. prep motion time is basically ignored...
                                            0.1,    0.1,      // lift
                                            0.15,   0.15,      // plant
                                            0.25,   0.25};     // return

creepTimingParameters SLOW_CREEP_PARAMETERS {0,     0,    // prepare
                                            .1,     .1,      // lift
                                            .15,    .15,      // plant
                                            .5,     .5};     // return

class CreepCommanderContinuousCOM : public CreepCommander {
private:
		bool using_slow_gait = true;

public:
	CreepCommanderContinuousCOM(Dog *dog) {
		super(dog);
		timingParameters = SLOW_CREEP_PARAMETERS;
	}

	footCommand getNextFootCommand() {
		footCommand next_foot_command;
		next_foot_command.foot_to_move = chooseFootToMove();
		next_foot_command.foot_anchor_before_motion_oC = calculateNextLiftedFootAnchorBeforeMotion_fFoC();
        return next_foot_command;

	}

	COMCommand getNextCOMCommand() {
		COMCommand next_COM_command;
		next_COM_command.desired_COM_location = default_body_position;
        next_COM_command.motion_time = getTimingParameters().getTotalStepTime();
        return next_COM_command;
	}

    
};

#endif