#ifndef __CREEP__
#define __CREEP__

#include "Dog.h"
#include "CreepStateInfo.h"
#include "CreepGaitcoordinator.h"
#include "CreepTimingParameters.h"
#include "DesiredCreepMotionCommand.h"


#define LIFT_HEIGHT 30

#define ROTATION_LEG_INPUT_MULTIPLIER 3

#define CALL_MEMBER_FN(object,ptrToMember)  ((object)->*(ptrToMember))


class CreepGaitStateMachine {
    enum ActionMode {STARTUP, NORMAL, END};
    typedef void (CreepGaitStateMachine::*actionFunction)(ActionMode);
    RobotDog *dog;

    DesiredCreepMotionCommand desired_motion;
    Rot body_orientation_modification;
    int state;
    #define NUM_CREEP_STATES 5
    CreepStateInfo creep_states[NUM_CREEP_STATES];
    actionFunction actions[NUM_CREEP_STATES];

    Rot current_rotation = ROT_ZERO;
    Point current_centroid_offset_from_original_centroid = POINT_ZERO;
    Point next_foot_position_oBfF;
    Point lift_foot_position_oBfF;
    Point current_body_position_goal_oCfF;

    bool tracking_external_orientation = false;
    bool move_in_ground_frame = false;

    CreepGaitCoordinator *gait_coordinator;

    footCommand nextFootCommand;
    COMCommand nextCOMCommand;

public:
    CreepGaitStateMachine(RobotDog *ndog_r, CreepGaitCoordinator *ngait_coordinator) {
        dog = ndog_r;
        gait_coordinator = ngait_coordinator;

        CreepTimingParameters timing_parameters = gait_coordinator->getTimingParameters(); // TODO this gets set on declaration...cant switch tmiing from coordinator.
        current_body_position_goal_oCfF = gait_coordinator->getDefaultBodyPosition();

        state = 0;
        actions[0]      = &CreepGaitStateMachine::waitForMotionCommand;
        actions[1]      = &CreepGaitStateMachine::prepareCOM;
        actions[2]      = &CreepGaitStateMachine::liftFoot;
        actions[3]      = &CreepGaitStateMachine::plantFoot;
        actions[4]      = &CreepGaitStateMachine::returnCOM;
        creep_states[0] = CreepStateInfo(0, 0, TIME_INFINITE);
        creep_states[1] = CreepStateInfo(1, timing_parameters.prepare_motion_time, 
                                            timing_parameters.prepare_state_time);
        creep_states[2] = CreepStateInfo(2, timing_parameters.lift_motion_time, 
                                            timing_parameters.lift_state_time);
        creep_states[3] = CreepStateInfo(3, timing_parameters.plant_motion_time, 
                                            timing_parameters.plant_state_time);
        creep_states[4] = CreepStateInfo(4, timing_parameters.return_motion_time, 
                                            timing_parameters.return_state_time);
    }

    void operate() {

        if (getCurrentCreepState()->isInStartup()) {
            Serial.println(state);
            doCurrentStateAction(STARTUP);
            getCurrentCreepState()->setStarted();
            Serial.println("startup done");
        }
        
        doCurrentStateAction(NORMAL);

        if (tracking_external_orientation) {
            trackExternalOrientation();
        }

        if (getCurrentCreepState()->isTimeToEnd()) {
            Serial.println("end beginning");
            doCurrentStateAction(END);
            int next_state = getCurrentCreepState()->getNextState(); 
            getCurrentCreepState()->reset();
            switchToState(next_state);
            Serial.println("ended");
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
            Serial.println("asdf0");
            Point body_position_from_original_centroid = -(dog->getBodyPositionFromCentroid(Frame::FLOOR));
            body_position_from_original_centroid.print();
            dog->switchFootStance(nextFootCommand.foot_to_move, FootStance::SET);
            Point body_position_from_swing_phase_centroid = -(dog->getBodyPositionFromCentroid(Frame::FLOOR));
            current_centroid_offset_from_original_centroid = body_position_from_swing_phase_centroid - body_position_from_original_centroid;

            current_centroid_offset_from_original_centroid.print();
            
            current_body_position_goal_oCfF = nextCOMCommand.desired_COM_from_original_centroid_oC;
            current_body_position_goal_oCfF.print();
            float time = nextCOMCommand.motion_time;

            if (move_in_ground_frame) {
                dog->moveBodyToPositionFromCentroidInTime(current_body_position_goal_oCfF, Frame::GROUND, time);
            } else {
                current_rotation += desired_motion.rotation; 
                dog->moveBodyToPositionFromCentroidInTime(current_body_position_goal_oCfF, Frame::FLOOR, time);
                dog->moveBodyToOrientationInTime(current_rotation + body_orientation_modification, time); // will this collide with trackExternalOrientation?
            }
        } else if (mode == NORMAL) {

        } else {
            getCurrentCreepState()->setNextState(2);
        }
    }

    void liftFoot(ActionMode mode) {
        if (mode == STARTUP) {
            Point lift_height = Point(0,0,LIFT_HEIGHT);
            lift_foot_position_oBfF = (dog->getFootPositionFromBody(nextFootCommand.foot_to_move, Frame::FLOOR) + lift_height);
            
            float time = getCurrentCreepState()->getMotionPeriod();

            dog->switchFootStance(nextFootCommand.foot_to_move, FootStance::LIFTED);
            dog->moveFootToPositionFromBodyInTime(nextFootCommand.foot_to_move, lift_foot_position_oBfF, Frame::FLOOR, time);        
        } else if (mode == NORMAL) {
            if (tracking_external_orientation) {
                dog->moveFootToPositionFromBody(nextFootCommand.foot_to_move, lift_foot_position_oBfF, Frame::FLOOR);
            }
        } else {
            getCurrentCreepState()->setNextState(3);
        }
    }

    void plantFoot(ActionMode mode) {
        if (mode == STARTUP) {
            Point centroid_position = -dog->getBodyPositionFromCentroid(Frame::FLOOR);
            next_foot_position_oBfF = (centroid_position + nextFootCommand.desired_foot_anchor_from_original_centroid_oC) - current_centroid_offset_from_original_centroid;

            float time = getCurrentCreepState()->getMotionPeriod();
            dog->moveFootToPositionFromBodyInTime(nextFootCommand.foot_to_move, next_foot_position_oBfF, Frame::FLOOR, time);
        } else if (mode == NORMAL) {
            if (tracking_external_orientation) {
                dog->moveFootToPositionFromBody(nextFootCommand.foot_to_move, next_foot_position_oBfF, Frame::FLOOR);
            }
        } else {
            dog->switchFootStance(nextFootCommand.foot_to_move, FootStance::PLANTED);
            current_body_position_goal_oCfF = dog->getBodyPositionFromCentroid(Frame::FLOOR);
            getCurrentCreepState()->setNextState(0);
            if (move_in_ground_frame) {
                current_rotation += desired_motion.rotation; 
            }
        }
    }


    void returnCOM(ActionMode mode) {
        if (mode == STARTUP) {
            float time = getCurrentCreepState()->getMotionPeriod();

            current_body_position_goal_oCfF = gait_coordinator->getDefaultBodyPosition();

            if (move_in_ground_frame) {
                dog->moveBodyToPositionFromCentroidInTime(current_body_position_goal_oCfF, Frame::GROUND, time);
            } else {
                dog->moveBodyToPositionFromCentroidInTime(current_body_position_goal_oCfF, Frame::FLOOR, time);
                dog->moveBodyToOrientationInTime(current_rotation + body_orientation_modification, time);
            }
        } else if (mode == NORMAL) {

        } else {
            getCurrentCreepState()->setNextState(0);
        }
    }

public:
    // x y yaw are desired centroid locations after motion
    void sendMotionCommand(float x_motion, float y_motion, float yaw_motion) {
        if (state == 0) {
            desired_motion.translation = Point(x_motion, y_motion, 0);
            desired_motion.rotation = Rot(0, 0, yaw_motion);
            desired_motion.leg_rotation = desired_motion.rotation * ROTATION_LEG_INPUT_MULTIPLIER;

            getCurrentCreepState()->endEarly();
            getCurrentCreepState()->setNextState(1);

            //calculate desired foot, motion, etc
            gait_coordinator->receiveDesiredMotion(desired_motion);
            nextFootCommand = gait_coordinator->getNextFootCommand();
            nextCOMCommand = gait_coordinator->getNextCOMCommand();
        }
    }

    // void sendMotionCommandUsingFoot(int foot_i, float x_motion, float y_motion, float yaw_motion) {
    //     if (state == 0) {
    //         desired_motion.translation = Point(x_motion, y_motion, 0);
    //         desired_motion.rotation = Rot(0, 0, yaw_motion);
    //         desired_motion.leg_rotation = desired_motion.rotation * ROTATION_LEG_INPUT_MULTIPLIER;

    //         getCurrentCreepState()->endEarly();
    //         getCurrentCreepState()->setNextState(1);

    //         overrideFootChoice = true;
    //         nextFootCommand.foot_to_move = foot_i;
    //         //nextFootCommand.foot_anchor = foot_i;
    //     }
    // }

    void sendReturnToCOMCommand() {
        if (state == 0) {
            getCurrentCreepState()->endEarly();
            getCurrentCreepState()->setNextState(4); 
        }
    }


    bool isWaiting() {
        return (state == 0);
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

    Rot getCurrentRotation() {
        return current_rotation;
    }
};


#endif
