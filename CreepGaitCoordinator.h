#ifndef __CREEP__
#define __CREEP__

#include "Dog.h"
#include "CreepStateInfo.h"

#define LIFT_HEIGHT 30
#define BODY_DEFAULT_HEIGHT 110

#define STATE_PREPARE_TIME 0.35
#define STATE_PREPARE_OVERLAP_FACTOR 0.6
#define STATE_LIFT_TIME 0.1
#define STATE_LIFT_OVERLAP_FACTOR 0
#define STATE_PLANT_TIME 0.15
#define STATE_PLANT_OVERLAP_FACTOR 0.0

#define STATE_RETURN_TIME 0.25
#define STATE_RETURN_OVERLAP_FACTOR 0

#define ROTATION_LEG_INPUT_MULTIPLIER 3


#define CALL_MEMBER_FN(object,ptrToMember)  ((object)->*(ptrToMember))

class CreepGaitCoordinator {
    enum ActionMode {STARTUP, NORMAL, END};
    typedef void (CreepGaitCoordinator::*actionFunction)(ActionMode);
    RobotDog *dog;

    struct CreepMotionCommand {
        Point translation;
        Rot rotation;
        Rot leg_rotation;

        CreepMotionCommand() {
            translation = POINT_ZERO;
            rotation = ROT_ZERO;
            leg_rotation = ROT_ZERO;
        }
    };

    CreepMotionCommand desired_motion;
    Rot body_orientation_modification;
    int state;
    #define NUM_CREEP_STATES 5
    CreepStateInfo creep_states[NUM_CREEP_STATES];
    actionFunction actions[NUM_CREEP_STATES];

    Rot current_rotation = ROT_ZERO;
    Point body_distance_from_original_centroid = POINT_ZERO;
    int foot_to_move = 0;
    Point next_foot_anchor_oC;

    bool overrideFootChoice = false;
    int overridden_foot_to_move = 0;
    Point default_body_position = Point(0, 0, BODY_DEFAULT_HEIGHT);
    int step_order[4] = {0, 2, 3, 1};
    int step_order_iterator = 3;

public:
    CreepGaitCoordinator(RobotDog *n_dog) {
        dog = n_dog;
        state = 0;
        actions[0]      = &CreepGaitCoordinator::waitForMotionCommand;
        actions[1]      = &CreepGaitCoordinator::prepareCOM;
        actions[2]      = &CreepGaitCoordinator::liftFoot;
        actions[3]      = &CreepGaitCoordinator::plantFoot;
        actions[4]      = &CreepGaitCoordinator::returnCOM;
        creep_states[0] = CreepStateInfo(0, TIME_INFINITE);
        creep_states[1] = CreepStateInfo(1, STATE_PREPARE_TIME * (1-STATE_PREPARE_OVERLAP_FACTOR));
        creep_states[2] = CreepStateInfo(2, STATE_LIFT_TIME * (1-STATE_LIFT_OVERLAP_FACTOR));
        creep_states[3] = CreepStateInfo(3, STATE_PLANT_TIME * (1-STATE_PLANT_OVERLAP_FACTOR));
        creep_states[4] = CreepStateInfo(4, STATE_RETURN_TIME);
    }

    void operate() {
        if (getCurrentCreepState()->isInStartup()) {
            doCurrentStateAction(STARTUP);
            getCurrentCreepState()->setStarted();
        }
        
        doCurrentStateAction(NORMAL);
        
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
        //(*actions[state])(mode);
    }

    void switchToState(int n_state) {
        state = n_state;
    }

    void waitForMotionCommand(ActionMode mode) {
        // literally do nothing...
        if (mode == STARTUP) {
  
        } else if (mode == NORMAL) {
            //body move to desired orientation?
        } else {

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
            desired_motion.translation = Point(x_motion, y_motion, 0);
            desired_motion.rotation = Rot(0, 0, yaw_motion);
            desired_motion.leg_rotation = desired_motion.rotation * ROTATION_LEG_INPUT_MULTIPLIER;

            getCurrentCreepState()->setEnded();
            getCurrentCreepState()->setNextState(1);

            overrideFootChoice = true;
            overridden_foot_to_move = foot_i;
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

    bool isWaiting() {
        return (state == 0);
    }

    Rot getCurrentOrientation() {
        return current_rotation;
    }

private:
    void prepareCOM(ActionMode mode) {
        if (mode == STARTUP) {
            Point old_body_distance_from_planted_centroid = -(dog->getBodyPositionFromCentroid(Frame::FLOOR) - default_body_position);;
            
            foot_to_move = chooseFootToMove();
            next_foot_anchor_oC = calculateNextLiftedFootAnchor_fF(foot_to_move);
            
            dog->switchFootStance(foot_to_move, FootStance::SET);
            Point old_body_distance_from_set_centroid = -(dog->getBodyPositionFromCentroid(Frame::FLOOR) - default_body_position);
            body_distance_from_original_centroid = old_body_distance_from_set_centroid - old_body_distance_from_planted_centroid;
            
            float time = STATE_PREPARE_TIME;
            dog->moveBodyToPositionFromCentroid(default_body_position, Frame::FLOOR, time);
            dog->moveBodyToOrientation(current_rotation + desired_motion.rotation, time);
            current_rotation += desired_motion.rotation;    
        } else if (mode == NORMAL) {
        
        } else {
            getCurrentCreepState()->setNextState(2);
        }
    }

    void liftFoot(ActionMode mode) {
        if (mode == STARTUP) {
            Point lift_height = Point(0,0,LIFT_HEIGHT);
            Point next_foot_position_oBfF = (dog->getFootPositionFromBody(foot_to_move, Frame::FLOOR) + lift_height);
            
            float time = STATE_LIFT_TIME;
            dog->moveFootToPositionFromBody(foot_to_move, next_foot_position_oBfF, Frame::FLOOR, time);        
        } else if (mode == NORMAL) {
        
        } else {
            getCurrentCreepState()->setNextState(3);
        }
    }

    void plantFoot(ActionMode mode) {
        if (mode == STARTUP) {
            Point centroid_position = -dog->getBodyPositionFromCentroid(Frame::FLOOR);
            Point next_foot_position_oBfF = (centroid_position + next_foot_anchor_oC) - body_distance_from_original_centroid;

            float time = STATE_PLANT_TIME;
            dog->moveFootToPositionFromBody(foot_to_move, next_foot_position_oBfF, Frame::FLOOR, time);
        } else if (mode == NORMAL) {
        
        } else {
            dog->switchFootStance(foot_to_move, FootStance::PLANTED);
            getCurrentCreepState()->setNextState(0);
        }
    }

    // TODO: incorporate size of support polygon into consideration...dog is tripping over itself
    int chooseFootToMove() {
        if (overrideFootChoice) {
            overrideFootChoice = false;
            return overridden_foot_to_move;
        }
        float max_distance = 0;
        int foot_to_move = 0;
        for (int i = 0; i < NUM_LEGS; i++) {
            Point current_foot_anchor_fFoC = dog->getFootPositionFromBody(i, Frame::FLOOR) - dog->getCentroidPositionFromBody(Frame::FLOOR);
            Point next_foot_anchor_if_not_lifted = current_foot_anchor_fFoC - dog->rotateFromFrame(desired_motion.translation, current_rotation + desired_motion.rotation);
            Point lifted_anchor_deviation_from_unlifted = next_foot_anchor_if_not_lifted - calculateNextLiftedFootAnchor_fF(i);
            float distance = lifted_anchor_deviation_from_unlifted.norm();
            if (distance > max_distance) {
               foot_to_move = i;
               max_distance = distance;
            }
        }
       return foot_to_move;
        // step_order_iterator = (step_order_iterator + 1)%4;
        // return step_order[step_order_iterator];
    }

    Point calculateNextLiftedFootAnchor_fF(int foot_i) {
        Point default_anchor = dog->getDefaultFootPosition(foot_i, Frame::BODY) + Point(0, 0, dog->getStartingHeight());
        return (default_anchor + desired_motion.translation) * (current_rotation + desired_motion.leg_rotation); // body frame movement
    }

    void returnCOM(ActionMode mode) {
        if (mode == STARTUP) {
            
            float time = STATE_RETURN_TIME;
            dog->moveBodyToPositionFromCentroid(default_body_position, Frame::FLOOR, time);
            dog->moveBodyToOrientation(current_rotation, time);
            current_rotation += desired_motion.rotation;        
        } else if (mode == NORMAL) {
        
        } else {
            getCurrentCreepState()->setNextState(0);
        }
    }
};


#endif
