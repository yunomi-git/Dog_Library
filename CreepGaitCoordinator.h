#ifndef __CREEP__
#define __CREEP__

#include "Dog.h"
#include "CreepStateInfo.h"
#include "Triangle.h"

#define LIFT_HEIGHT 30
#define BODY_DEFAULT_HEIGHT 110


#ifdef FAST_CREEP
#define STATE_PREPARE_TIME 0.35
#define STATE_PREPARE_OVERLAP_FACTOR 0.6
#define STATE_LIFT_TIME 0.1
#define STATE_LIFT_OVERLAP_FACTOR 0
#define STATE_PLANT_TIME 0.15
#define STATE_PLANT_OVERLAP_FACTOR 0.0

#define STATE_RETURN_TIME 0.25
#define STATE_RETURN_OVERLAP_FACTOR 0

#else
#define STATE_PREPARE_TIME 0.7
#define STATE_PREPARE_OVERLAP_FACTOR 0
#define STATE_LIFT_TIME 0.1
#define STATE_LIFT_OVERLAP_FACTOR 0
#define STATE_PLANT_TIME 0.15
#define STATE_PLANT_OVERLAP_FACTOR 0.0

#define STATE_RETURN_TIME 0.5
#define STATE_RETURN_OVERLAP_FACTOR 0
#endif

#define ROTATION_LEG_INPUT_MULTIPLIER 3

#define CALL_MEMBER_FN(object,ptrToMember)  ((object)->*(ptrToMember))

#define DEFAULT_INSCRIBED_CIRCLE_AREA 8439.56
#define MAX_LOST_LEG_DISTANCE 100

#define LOST_DISTANCE_WEIGHT 1
#define NEXT_AREA_WEIGHT 0.5
#define CURR_AREA_WEIGHT 1

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

    Point default_body_position = Point(0, 0, BODY_DEFAULT_HEIGHT);
    Rot current_rotation = ROT_ZERO;
    Point body_distance_from_original_centroid = POINT_ZERO;
    int foot_to_move = 0;
    Point next_foot_anchor_oC;
    Point next_foot_position_oBfF;
    Point current_body_position_goal_oCfF = default_body_position;

    bool tracking_external_orientation = false;
    bool move_in_ground_frame = false;
    bool overrideFootChoice = false;
    int overridden_foot_to_move = 0;
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

        // setSpeed;
        // setSpeed;
    }

    void operate() {
        if (getCurrentCreepState()->isInStartup()) {
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
            Point old_body_distance_from_planted_centroid = -(dog->getBodyPositionFromCentroid(Frame::FLOOR) - default_body_position);;
            
            foot_to_move = chooseFootToMove();
            next_foot_anchor_oC = calculateNextLiftedFootAnchor_fFoC(foot_to_move);
            
            dog->switchFootStance(foot_to_move, FootStance::SET);

            Point old_body_distance_from_set_centroid = -(dog->getBodyPositionFromCentroid(Frame::FLOOR) - default_body_position);
            body_distance_from_original_centroid = old_body_distance_from_set_centroid - old_body_distance_from_planted_centroid;
            
            current_body_position_goal_oCfF = default_body_position;

            float time = STATE_PREPARE_TIME;
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
            Point lift_height = Point(0,0,LIFT_HEIGHT);
            next_foot_position_oBfF = (dog->getFootPositionFromBody(foot_to_move, Frame::FLOOR) + lift_height);
            
            float time = STATE_LIFT_TIME;

            dog->switchFootStance(foot_to_move, FootStance::LIFTED);
            dog->moveFootToPositionFromBodyInTime(foot_to_move, next_foot_position_oBfF, Frame::FLOOR, time);        
        } else if (mode == NORMAL) {
            if (tracking_external_orientation) {
                dog->moveFootToPositionFromBody(foot_to_move, next_foot_position_oBfF, Frame::FLOOR);
            }
        } else {
            getCurrentCreepState()->setNextState(3);
        }
    }

    void plantFoot(ActionMode mode) {
        if (mode == STARTUP) {
            Point centroid_position = -dog->getBodyPositionFromCentroid(Frame::FLOOR);
            next_foot_position_oBfF = (centroid_position + next_foot_anchor_oC) - body_distance_from_original_centroid;

            float time = STATE_PLANT_TIME;
            dog->moveFootToPositionFromBodyInTime(foot_to_move, next_foot_position_oBfF, Frame::FLOOR, time);
        } else if (mode == NORMAL) {
            if (tracking_external_orientation) {
                dog->moveFootToPositionFromBody(foot_to_move, next_foot_position_oBfF, Frame::FLOOR);
            }
        } else {
            dog->switchFootStance(foot_to_move, FootStance::PLANTED);
            current_body_position_goal_oCfF = dog->getBodyPositionFromCentroid(Frame::FLOOR);
            getCurrentCreepState()->setNextState(0);
            if (move_in_ground_frame) {
                current_rotation += desired_motion.rotation; 
            }
        }
    }

    // TODO: incorporate size of support polygon into consideration...dog is tripping over itself
    // deviation from original polygon size?
    // negatively penalize a negative foot area?
    // maximize next 3-foot area combinations
    int chooseFootToMove() {
        if (overrideFootChoice) {
            overrideFootChoice = false;
            return overridden_foot_to_move;
        }

        float max_move_score = 0;
        int foot_to_move = 0;
        for (int i = 0; i < NUM_LEGS; i++) {
            float move_score = calculateFootLiftChoiceScore(i);
            if (move_score > max_move_score) {
               foot_to_move = i;
               max_move_score = move_score;
            }
        }
       return foot_to_move;
    }

    float calculateFootLiftChoiceScore(int foot_i) {
        float lost_distance_from_not_moving;
        Point current_anchor_fFoC = dog->getFootPositionFromBody(foot_i, Frame::FLOOR) - dog->getCentroidPositionFromBody(Frame::FLOOR);
        Point next_anchor_fFoC = calculateNextLiftedFootAnchor_fFoC(foot_i);
        lost_distance_from_not_moving = (next_anchor_fFoC - current_anchor_fFoC).norm();
        float normalized_lost_distance = lost_distance_from_not_moving/MAX_LOST_LEG_DISTANCE;
        if (normalized_lost_distance > 1) {
            normalized_lost_distance *= 3;
        }

        Triangle all_support_polygons[4];
        getSupportPolygonsAfterFootMotion(all_support_polygons, foot_i);
        float min_next_inscribed_circle_area;
        Triangle support_poly_with_footi[NUM_LEGS - 1] = {all_support_polygons[0], all_support_polygons[1], all_support_polygons[2]};
        min_next_inscribed_circle_area = TriangleSet::getMinimumInscribedCircleArea(support_poly_with_footi, NUM_LEGS - 1);
        float normalized_next_circle_area = min_next_inscribed_circle_area/(DEFAULT_INSCRIBED_CIRCLE_AREA * 1.5);
        if (normalized_next_circle_area > 1) {
            normalized_next_circle_area = 1;
        }

        float supporting_inscribed_circle_area;
        Triangle support_poly_without_footi = all_support_polygons[3];
        supporting_inscribed_circle_area = Triangle::getInscribedCircleArea(support_poly_without_footi);
        float normalized_support_circle_area = supporting_inscribed_circle_area/DEFAULT_INSCRIBED_CIRCLE_AREA;
        if (normalized_support_circle_area > 1) {
            normalized_support_circle_area = 1;
        }

        float score = LOST_DISTANCE_WEIGHT * normalized_lost_distance + // larger distance is higher priority
                      NEXT_AREA_WEIGHT     * normalized_next_circle_area +  // smaller area is lower priority
                      CURR_AREA_WEIGHT     * normalized_support_circle_area; // larger area is higher priority

#ifdef DEBUG
        Serial.print(foot_i); Serial.print(": "); 
        Serial.print("dist: "); Serial.print(normalized_lost_distance);
        Serial.print(", min: "); Serial.print(normalized_next_circle_area);
        Serial.print(", curr: "); Serial.print(normalized_support_circle_area);
        Serial.print(" score: "); Serial.print(score);
        Serial.println();
#endif
        return score;
    }

    // from CURRENT centroid
    Point calculateNextLiftedFootAnchor_fFoC(int foot_i) {
        Point default_anchor = dog->getDefaultFootPosition(foot_i, Frame::BODY) + Point(0, 0, dog->getStartingHeight());
        return (default_anchor + desired_motion.translation) * (current_rotation + desired_motion.leg_rotation); // body frame movement
    }

    // places a list of 4 into support_polygons
    // polygon that does not include foot_i is the last
    void getSupportPolygonsAfterFootMotion(Triangle *support_polygons, int foot_i) {
        Point foot_positions[NUM_LEGS];
        for (int i = 0; i < NUM_LEGS; i++) {
            if (i == foot_i) {
                foot_positions[i] = calculateNextLiftedFootAnchor_fFoC(i);
            } else {
                foot_positions[i] = dog->getAnchorPoint_oC(i, Frame::FLOOR); 
            }
        }

        int poly_without_footi_counted = 0;
        for (int f1 = 0; f1 < NUM_LEGS - 2; f1++) {
            for (int f2 = f1 + 1; f2 < NUM_LEGS - 1; f2++) {
                for (int f3 = f2 + 1; f3 < NUM_LEGS; f3++) {
                    if ((f1 != foot_i) && (f2 != foot_i) && (f3 != foot_i)) {
                        support_polygons[NUM_LEGS - 1] = Triangle(foot_positions[f1], foot_positions[f2], foot_positions[f3]);
                    } else {
                        support_polygons[poly_without_footi_counted] = Triangle(foot_positions[f1], foot_positions[f2], foot_positions[f3]);
                        poly_without_footi_counted++;
                    }
                }
            }
        }
    }

    void returnCOM(ActionMode mode) {
        if (mode == STARTUP) {
            float time = STATE_RETURN_TIME;

            current_body_position_goal_oCfF = default_body_position;

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
