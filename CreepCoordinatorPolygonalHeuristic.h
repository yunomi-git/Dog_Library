#ifndef __CREEPCOMMANDERPOLYGONAL__
#define __CREEPCOMMANDERPOLYGONAL__

#include "Triangle.h"
#include "CreepGaitCoordinator.h"

#define DEFAULT_INSCRIBED_CIRCLE_AREA 8439.56
#define MAX_LOST_LEG_DISTANCE 100

#define LOST_DISTANCE_WEIGHT 1
#define NEXT_AREA_WEIGHT 0.5
#define CURR_AREA_WEIGHT 1



class CreepCoordinatorPolygonal : public CreepGaitCoordinator {
private:
    // creepTimingParameters FAST_TROT_PARAMETERS {0.35,    0.6,    // prepare
    //                                          0.1,    0,      // lift
    //                                          0.15,   0,      // plant
    //                                          0.25,   0};     // return

    // creepTimingParameters SLOW_TROT_PARAMETERS {.7,  0.0,    // prepare
    //                                          .1,     0,      // lift
    //                                          .15,    0,      // plant
    //                                          .5,     0};     // return

    const CreepTimingParameters FAST_CREEP_PARAMETERS {0.35,   0.14,    // prepare
                                                0.1,    0.1,      // lift
                                                0.15,   0.15,      // plant
                                                0.25,   0.25};     // return

    const CreepTimingParameters SLOW_CREEP_PARAMETERS {.7,     0.7,    // prepare
                                                .1,     .1,      // lift
                                                .15,    .15,      // plant
                                                .5,     .5};     // return

    Point default_body_position = Point(0, 0, BODY_DEFAULT_HEIGHT);

public: 
    CreepCoordinatorPolygonal(RobotDog *ndog) : CreepGaitCoordinator(ndog) {
        timingParameters = FAST_CREEP_PARAMETERS;
    }
    
	void useSlowGait() {
		timingParameters = SLOW_CREEP_PARAMETERS;
	}

	void useFastGait() {
		timingParameters = FAST_CREEP_PARAMETERS;
	}

	footCommand getNextFootCommand() {
		footCommand next_foot_command;
		next_foot_command.foot_to_move = chooseFootToMove();
		next_foot_command.desired_foot_anchor_from_original_centroid_oC = calculateNextLiftedFootAnchorFromOriginalCentroid_fFoC(next_foot_command.foot_to_move);
        return next_foot_command;

	}

	COMCommand getNextCOMCommand() {
		COMCommand next_COM_command;
		next_COM_command.desired_COM_from_original_centroid_oC = default_body_position;
        next_COM_command.motion_time = getTimingParameters().prepare_motion_time;
        return next_COM_command;
	}

// TODO: incorporate size of support polygon into consideration...dog is tripping over itself
    // deviation from original polygon size?
    // negatively penalize a negative foot area?
    // maximize next 3-foot area combinations
    int chooseFootToMove() {
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
        Point current_anchor_fFoC = dog_r->getFootPositionFromBody(foot_i, Frame::FLOOR) - dog_r->getCentroidPositionFromBody(Frame::FLOOR);
        Point next_anchor_fFoC = calculateNextLiftedFootAnchorFromOriginalCentroid_fFoC(foot_i);
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
    Point calculateNextLiftedFootAnchorFromOriginalCentroid_fFoC(int foot_i) {
        Point default_anchor = dog_r->getDefaultFootPosition(foot_i, Frame::BODY) + Point(0, 0, dog_r->getStartingHeight());
        return (default_anchor + desired_motion.translation) * (desired_motion.current_rotation + desired_motion.leg_rotation); // body frame movement
    }

    // places a list of 4 into support_polygons
    // polygon that does not include foot_i is the last
    void getSupportPolygonsAfterFootMotion(Triangle *support_polygons, int foot_i) {
        Point foot_positions[NUM_LEGS];
        for (int i = 0; i < NUM_LEGS; i++) {
            if (i == foot_i) {
                foot_positions[i] = calculateNextLiftedFootAnchorFromOriginalCentroid_fFoC(i);
            } else {
                foot_positions[i] = dog_r->getAnchorPoint_oC(i, Frame::FLOOR); 
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

};

#endif