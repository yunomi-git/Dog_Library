#ifndef __CREEP__
#define __CREEP__

#include "Dog.h"


#define LIFT_HEIGHT 30

#define TRANSLATION_INPUT_SCALING 40
#define TRANSLATION_INPUT_THRESHOLD 5
#define ROTATION_INPUT_SCALING 8
#define ROTATION_LEG_INPUT_MULTIPLIER 3

#define STATE_PREPARE_TIME 0.5
#define STATE_PREPARE_OVERLAP_FACTOR 0.4
#define STATE_LIFT_TIME 0.1
#define STATE_LIFT_OVERLAP_FACTOR 0
#define STATE_PLANT_TIME 0.2
#define STATE_PLANT_OVERLAP_FACTOR 0.7
#define STATE_RETURN_TIME 0.25
#define STATE_RETURN_OVERLAP_FACTOR 0


class CreepGaitCoordinator {

    Dog *dog;

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

    Rot current_rotation = ROT_ZERO;
    Point body_distance_from_original_centroid = POINT_ZERO;
    int foot_to_move = 0;
    Point next_foot_anchor_oC;

    Point default_body_position = Point(0, 0, 100);
    int step_order[4] = {0, 2, 3, 1};
    int step_order_iterator = 3;

public:
    CreepGaitCoordinator() {
        state = 0;
        creep_state_info[0] = CreepStateInfo(waitForMotionCommand, 0, TIME_INFINITE);
        creep_state_info[1] = CreepStateInfo(prepareCOM,           1, STATE_PREPARE_TIME * (1-STATE_PREPARE_OVERLAP_FACTOR));
        creep_state_info[2] = CreepStateInfo(liftFoot,             2, STATE_LIFT_TIME * (1-STATE_LIFT_OVERLAP_FACTOR),       );
        creep_state_info[3] = CreepStateInfo(plantFoot,            3, STATE_PLANT_TIME * (1-STATE_PLANT_OVERLAP_FACTOR));
        creep_state_info[4] = CreepStateInfo(returnCOM,            4, STATE_RETURN_TIME);
    }

    operate() {
        if (getCurrentCreepState()->is_in_startup) {
            getCurrentCreepState()->startupAction();
            getCurrentCreepState()->setStarted();
        }
        
        getCurrentCreepState()->normalAction();
        
        if (getCurrentCreepState()->isTimeToEnd()) {
            getCurrentCreepState()->endAction();
            getCurrentCreepState()->reset();
            switchToState(getCurrentCreepState()->getNextState());
        }
    }

    CreepStateInfo *getCurrentCreepState() {
        return (creep_states + state); // state is the index
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

    void sendMotionCommand(float x_motion, float y_motion, float yaw_motion) {
        if (state == 0) {
            desired_motion.translation = Point(x_motion, y_motion, 0);
            desired_motion.rotation = Rot(0, 0, yaw_motion);
            desired_motion.leg_rotation = desired_motion.rotation * ROTATION_LEG_INPUT_MULTIPLIER;

            getCurrentCreepState().end();
            getCurrentCreepState()->setNextState(1);
        }
    }

    void sendReturnToCOMCommand() {
        if (state == 0) {
            getCurrentCreepState().end();
            getCurrentCreepState()->setNextState(4); 
        }
    }

    void modifyBodyOrientation(Rot orientation_modification) {
        body_orientation_modification = orientation_modification;
    }


    void prepareCOM(ActionMode mode) {
        if (mode == STARTUP) {
            Point old_body_distance_from_planted_centroid = -(dog->getBodyPositionFromCentroid(Frame::FLOOR) - default_body_position);;
            
            foot_to_move = chooseFootToMove();
            next_foot_anchor_oC = calculateNextFootAnchor_fF(foot_to_move);
            
            dog->switchFootStance(foot_to_move, FootStance::SET);
            Point old_body_distance_from_set_centroid = -(dog->getBodyPositionFromCentroid(Frame::FLOOR) - default_body_position);
            body_distance_from_original_centroid = old_body_distance_from_set_centroid - old_body_distance_from_planted_centroid;
            
            float time = getCurrentCreepState()->getActionPeriod();
            dog->moveBodyToPositionFromCentroid(default_body_position, Frame::FLOOR, time);
            dog->moveBodyToOrientation(current_rotation + desired_rotation, time);
            current_rotation += desired_rotation;    
        } else if (mode == NORMAL) {
        
        } else {
            getCurrentCreepState()->setNextState(2);
        }
    }

    void liftFoot(ActionMode mode) {
        if (mode == STARTUP) {
            Point lift_height = Point(0,0,LIFT_HEIGHT);
            Point next_foot_position_oBfF = (dog->getFootPositionFromBody(foot_to_move, Frame::FLOOR) + lift_height);
            
            float time = getCurrentCreepState()->getActionPeriod();
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

            float time = getCurrentCreepState()->getActionPeriod();
            dog->moveFootToPositionFromBody(foot_to_move, next_foot_position_oBfF, Frame::FLOOR, time);
        } else if (mode == NORMAL) {
        
        } else {
            dog->switchFootStance(foot_to_move, FootStance::PLANTED);
            getCurrentCreepState()->setNextState(0);
        }
    }

    int chooseFootToMove() {
    //    float max_distance = 0;
    //    int foot_to_move = 0;
    //    for (int i = 0; i < NUM_LEGS; i++) {
    //        Point next_foot_position = dog->getFootPositionFromBody(i, FRAME::BODY) * desired_rotation + desired_translation;
    //        Point deviation_from_anchor = next_foot_position - calculateNextFootAnchor_fF(i);// needs to be written
    //        float distance = deviation_from_anchor.norm();
    //        if (distance < max_distance) {
    //            foot_to_move = i;
    //            max_distance = distance;
    //        }
    //    }
    //    return foot_to_move;
        step_order_iterator = (step_order_iterator + 1)%4;
        return step_order[step_order_iterator];
    }

    Point calculateNextFootAnchor_fF(int foot_i) {
        Point default_anchor = dog->getDefaultFootPosition(foot_i, Frame::BODY) + Point(0, 0, dog->getStartingHeight());
        return (default_anchor + desired_motion.translation) * (current_rotation + desired_motion.leg_rotation); // body frame movement
    }

    void returnCOM(ActionMode mode) {
        if (mode == STARTUP) {
            dog->switchFootStance(foot_to_move, FootStance::PLANTED);
            Point default_height = Point(0,0,100);
            
            float time = getCurrentCreepState()->getActionPeriod();
            dog->moveBodyToPositionFromCentroid(default_height, Frame::FLOOR, time);
            dog->moveBodyToOrientation(current_rotation + desired_rotation, time);
            current_rotation += desired_rotation;        
        } else if (mode == NORMAL) {
        
        } else {
            getCurrentCreepState()->setNextState(0);
        }
    }
};


#endif
