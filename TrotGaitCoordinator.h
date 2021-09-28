#ifndef __TROTGAITCOORDINATOR__
#define __TROTGAITCOORDINATOR__

#include "Dog.h"
#include "CreepStateInfo.h"
#include "GaitParameters.h"

class TrotGaitCoordinator {
    enum ActionMode {STARTUP, NORMAL, END};
    typedef void (TrotGaitCoordinator::*actionFunction)(ActionMode);
    RobotDog *dog;

    struct MotionCommand {
        Point translation;
        Rot rotation;
        Rot leg_rotation;

        MotionCommand() {
            translation = POINT_ZERO;
            rotation = ROT_ZERO;
            leg_rotation = ROT_ZERO;
        }
    };

    MotionCommand desired_motion;
    
    int state = 0;
    #define NUM_CREEP_STATES 5
    CreepStateInfo creep_states[NUM_CREEP_STATES];
    actionFunction actions[NUM_CREEP_STATES];

    Point default_body_position = Point(0, 0, BODY_DEFAULT_HEIGHT);
    Rot current_rotation = ROT_ZERO;
    Point next_foot_anchor_oCfF[2] = {POINT_ZERO, POINT_ZERO};

    // External Modifications
    Point current_body_position_goal_oCfF = default_body_position;
    bool tracking_external_orientation = false;
    bool move_in_ground_frame = false;
    bool overrideFootChoice = false;
    Rot body_orientation_modification;
    int overridden_feet_to_move = 0;

    trotFootGroup step_order[2] = {trotFootGroup{0,2}, 
    					 		   trotFootGroup{1,3}};
    int step_order_iterator = 0;
    int feet_to_move[2] = {0,2};// bad

    trotTimingParameters trot_timing_parameters = SLOW_TROT_PARAMETERS;

public:
    TrotGaitCoordinator(RobotDog *n_dog) {
        dog = n_dog;
        state = 0;
        actions[0]      = &TrotGaitCoordinator::waitForMotionCommand;
        actions[1]      = &TrotGaitCoordinator::prepareCOM;
        actions[2]      = &TrotGaitCoordinator::liftFoot;
        actions[3]      = &TrotGaitCoordinator::plantFoot;
        actions[4]      = &TrotGaitCoordinator::returnCOM;
        creep_states[0] = CreepStateInfo(0, 0,
        									TIME_INFINITE);
        creep_states[1] = CreepStateInfo(1, trot_timing_parameters.prepare_time,
        									trot_timing_parameters.prepare_time * (1-trot_timing_parameters.prepare_overlap_factor));
        creep_states[2] = CreepStateInfo(2, trot_timing_parameters.lift_time, 
        									trot_timing_parameters.lift_time * (1-trot_timing_parameters.lift_overlap_factor));
        creep_states[3] = CreepStateInfo(3, trot_timing_parameters.plant_time,
        									trot_timing_parameters.plant_time * (1-trot_timing_parameters.plant_overlap_factor));
        creep_states[4] = CreepStateInfo(4, trot_timing_parameters.return_time,
        									trot_timing_parameters.return_time);
    }

    void operate() {
        if (getCurrentCreepState()->isInStartup()) {
        	Serial.print("Now in state: "); Serial.println(state);
            doCurrentStateAction(STARTUP);
            getCurrentCreepState()->setStarted();
        }
        
        doCurrentStateAction(NORMAL);

        if (tracking_external_orientation) {
            trackExternalOrientation();
        }

        if (getCurrentCreepState()->isTimeToEnd()) {
            doCurrentStateAction(END);
            int next_state = getCurrentCreepState()->getNextState(); 
            getCurrentCreepState()->reset();
            switchToState(next_state);
        }
    }

private:
    CreepStateInfo *getCurrentCreepState() {
        return (creep_states + state); // state is the index
    }

    void doCurrentStateAction(ActionMode mode) {
        CALL_MEMBER_FN(this, actions[state])(mode);
    }

    void switchToState(int n_state) {
        state = n_state;
    }

    void trackExternalOrientation() {
        dog->moveBodyToOrientationAtSpeed(current_rotation + body_orientation_modification, 20);
        dog->moveBodyToPositionFromCentroid(current_body_position_goal_oCfF, Frame::GROUND);    
    }

private:
    void waitForMotionCommand(ActionMode mode) {
        if (mode == STARTUP) {
  
        } else if (mode == NORMAL) {

        } else {

        }
    }

    void prepareCOM(ActionMode mode) {
        if (mode == STARTUP) {        
        	// cycle feet to move
            step_order_iterator = (step_order_iterator+1)%2;
            feet_to_move[0] = step_order[step_order_iterator].foot1;
            feet_to_move[1] = step_order[step_order_iterator].foot2;

            Serial.print("feet to move: "); Serial.print(feet_to_move[0]); Serial.print(", "); Serial.print(feet_to_move[1]); Serial.println();
            
            current_body_position_goal_oCfF = default_body_position; // for external tracking

            // begin the motion
            float time = getCurrentCreepState()->getMotionPeriod();
            if (move_in_ground_frame) {
                dog->moveBodyToPositionFromCentroidInTime(default_body_position, Frame::GROUND, time);
            } else {
                current_rotation += desired_motion.rotation; 
                dog->moveBodyToPositionFromCentroidInTime(default_body_position, Frame::FLOOR, time);
                dog->moveBodyToOrientationInTime(current_rotation + body_orientation_modification, time);
            }
        } else if (mode == NORMAL) {

        } else {
            getCurrentCreepState()->setNextState(2);
        }
    }

    void liftFoot(ActionMode mode) {
        if (mode == STARTUP) {
            dog->switchFootStance(feet_to_move[0], FootStance::LIFTED);
            dog->switchFootStance(feet_to_move[1], FootStance::LIFTED);
            
            float time = getCurrentCreepState()->getMotionPeriod();
            dog->moveFootToPositionFromBodyInTime(feet_to_move[0], calculateLiftedFootPosition_fFoB(feet_to_move[0]), Frame::FLOOR, time);
            dog->moveFootToPositionFromBodyInTime(feet_to_move[1], calculateLiftedFootPosition_fFoB(feet_to_move[1]), Frame::FLOOR, time);
        } else if (mode == NORMAL) {
         //    if (tracking_external_orientation) {
	        // 		dog->moveFootToPositionFromBodyInTime(feet_to_move[0], calculateLiftedFootPosition_fFoB(feet_to_move[0]), Frame::FLOOR, time);
         //    		dog->moveFootToPositionFromBodyInTime(feet_to_move[1], calculateLiftedFootPosition_fFoB(feet_to_move[1]), Frame::FLOOR, time);
	        // }
        } else {
            getCurrentCreepState()->setNextState(3);
        }
    }

    Point calculateLiftedFootPosition_fFoB(int foot_i) {
    	Point lift_height = Point(0,0,LIFT_HEIGHT);
		return dog->getFootPositionFromBody(foot_i, Frame::FLOOR) + lift_height;
    }

    void plantFoot(ActionMode mode) {
        if (mode == STARTUP) {
        	next_foot_anchor_oCfF[0] = calculatePlantedFootAnchor_fFoC(feet_to_move[0]);
        	next_foot_anchor_oCfF[1] = calculatePlantedFootAnchor_fFoC(feet_to_move[1]);

            float time = getCurrentCreepState()->getMotionPeriod();
	        dog->moveFootToPositionFromBodyInTime(feet_to_move[0], calculatePlantedFootAnchor_fFoB(feet_to_move[0]), Frame::FLOOR, time);
	        dog->moveFootToPositionFromBodyInTime(feet_to_move[1], calculatePlantedFootAnchor_fFoB(feet_to_move[1]), Frame::FLOOR, time);
        } else if (mode == NORMAL) {
    //         if (tracking_external_orientation) {
	   //          dog->moveFootToPositionFromBodyInTime(feet_to_move[0], calculatePlantedFootAnchor_fFoB(feet_to_move[0]), Frame::FLOOR, time);
	   //          dog->moveFootToPositionFromBodyInTime(feet_to_move[1], calculatePlantedFootAnchor_fFoB(feet_to_move[1]), Frame::FLOOR, time);
	   //      }
        } else {
            dog->switchFootStance(feet_to_move[0], FootStance::PLANTED);
            dog->switchFootStance(feet_to_move[1], FootStance::PLANTED);

            current_body_position_goal_oCfF = dog->getBodyPositionFromCentroid(Frame::FLOOR); // for external tracking

            getCurrentCreepState()->setNextState(0);
            if (move_in_ground_frame) {
                current_rotation += desired_motion.rotation; 
            }
        }
    }

    Point calculatePlantedFootAnchor_fFoB(int foot_i) {
    	Point centroid_position_fFoB = -dog->getBodyPositionFromCentroid(Frame::FLOOR);
    	return centroid_position_fFoB + calculatePlantedFootAnchor_fFoC(foot_i);
    }

    // from CURRENT centroid
    Point calculatePlantedFootAnchor_fFoC(int foot_i) {
        Point default_anchor = dog->getDefaultFootPosition(foot_i, Frame::BODY) + Point(0, 0, dog->getStartingHeight());
        return (default_anchor + desired_motion.translation) * (current_rotation + desired_motion.leg_rotation); // body frame movement
    }

    void returnCOM(ActionMode mode) {
        if (mode == STARTUP) {
            current_body_position_goal_oCfF = default_body_position; // for external tracking

			float time = getCurrentCreepState()->getMotionPeriod();
            if (move_in_ground_frame) {
                dog->moveBodyToPositionFromCentroidInTime(default_body_position, Frame::GROUND, time);
            } else {
                dog->moveBodyToPositionFromCentroidInTime(default_body_position, Frame::FLOOR, time);
                dog->moveBodyToOrientationInTime(current_rotation + body_orientation_modification, time);
            }
        } else if (mode == NORMAL) {

        } else {
            getCurrentCreepState()->setNextState(0);
        }
    }

public:
    void sendMotionCommand(float x_motion, float y_motion, float yaw_motion) {
        if (state == 0) {
            desired_motion.translation = Point(x_motion, y_motion, 0);
            desired_motion.rotation = Rot(0, 0, yaw_motion);
            desired_motion.leg_rotation = desired_motion.rotation * ROTATION_LEG_INPUT_MULTIPLIER;

            getCurrentCreepState()->setEnded();
            getCurrentCreepState()->setNextState(1);
        }
    }

    void sendMotionCommandUsingFoot(int foot_i, float x_motion, float y_motion, float yaw_motion) {
        if (state == 0) {
            // desired_motion.translation = Point(x_motion, y_motion, 0);
            // desired_motion.rotation = Rot(0, 0, yaw_motion);
            // desired_motion.leg_rotation = desired_motion.rotation * ROTATION_LEG_INPUT_MULTIPLIER;

            // getCurrentCreepState()->setEnded();
            // getCurrentCreepState()->setNextState(1);

            // overrideFootChoice = true;
            // overridden_feet_to_move = foot_i;
        }
    }

    void sendReturnToCOMCommand() {
        if (state == 0) {
            getCurrentCreepState()->setEnded();
            getCurrentCreepState()->setNextState(4); 
        }
    }

    void modifyBodyOrientation(Rot orientation_modification) {
        body_orientation_modification = orientation_modification;
    }

    void toggleFollowExternalOrientation() {
        tracking_external_orientation = !tracking_external_orientation;
    }

    void toggleMotionInGroundFrame() {
        move_in_ground_frame = !move_in_ground_frame;
    }

    bool isWaiting() {
        return (state == 0);
    }

    Rot getCurrentRotation() {
        return current_rotation;
    }
};

#endif
