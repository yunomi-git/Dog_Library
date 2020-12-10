#ifndef __DOG__
#define __DOG__

#include "RServoDriver.h"
#include "Leg.h"

#include <arduino.h>
#include "math2.h"
#include "Point.h"
#include "Rot.h"
#include "IMU.h"
#include "Timer.h"
#include "Trajectory.h"
#include "Enumerations.h"
#include "FootNote.h"

// #define DEBUG

// TODO: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// - clean up code, fix variable names
// - something is happening with body motion trajectory when rotation is occurring...find out why
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// USAGE DETAILS
// Nomenclature:
//  - Foot is used interchangeably with Leg. Foot is generally used to refer to the contact point.
//    Leg is generally used to refer to motions.
//
// FOOT STANCE
// - Foot may be set to various stances:
//   Planted   : Foot is constant in FLOOR frame (ie is in contact with a surface). Used in centroid calculation
//   Set       : Foot is constant in FLOOR frame (ie is in contact with a surface). NOT Used in centroid calculation
//   Lifted    : Foot is not in contact with the floor. 
//   (Anchored): Set or Planted
//
//
/* ============ FRAMES ==========================
 * GROUND - Static reference frame of the world.
 *Frame::BODY   - Frame attached to the body.
 * FLOOR  - Governed by foot placement wrt ground frame (ex. walking up a slope) 
 * All frames currently have body origin (9/9/20)
 *
 * Measurements:
 * -Frame::BODY wrt GROUND  : measured by IMU. "imu" orientation
 * -Frame::BODY wrt FLOOR   : "set" orientation, kinematically driven 
 * - FLOOR wrt GROUND : 1. (imu /? set) orientation = floor orientation <- using this: easiest to implement (see notes locomotion/centroid(1))
 *                      2. best fit through feet points in ground plane 
 * note: by default all orientations measured wrt ground.
 */

/**
* IMPLEMENTATION DETAILS
* - All points are in GROUND FRAME and RELATIVE TOFrame::BODY ORIGIN unless otherwise noted
* - o_f_: origin {body, centroid, shoulder}, frame {body, ground, floor}
* 
* - Convert ground to body frame: p_B = p_G / imu_orientation
* - Convert body to ground frame: p_G = p_B * (imu_orientation)
*
* - Convert frame A to B: p_B = p_A / A2B_orientation
* - Convert frame B to A: p_A = p_B * A2B_orientation
*
* Frame conventions:
* - Anchor points: Floor frame centroid origin. // anchor point (fFoC) + centroid (fFoB) = leg position (fFoB)
* - Centroid: Floor Frame (but use in ground frame) body origin
* - DogLeg input/return: Body frame body origin
* - Create a new variable whenever frame conversion occurs
*
* All set_* variables must be set whenever signals are sent
* 
*/


class RobotDog {
private:
    // ======================= CALIBRATION ===============================================
    #define LEG_UR_C_ANG_OFS  (0)
    #define LEG_UR_S_ANG_OFS  (3)
    #define LEG_UR_E_ANG_OFS  (-6)

    #define LEG_BR_C_ANG_OFS  (0)
    #define LEG_BR_S_ANG_OFS  (-3.5)
    #define LEG_BR_E_ANG_OFS  (-1)

    #define LEG_BL_C_ANG_OFS  (0)
    #define LEG_BL_S_ANG_OFS  (-1)
    #define LEG_BL_E_ANG_OFS  (5)

    #define LEG_UL_C_ANG_OFS  (0)
    #define LEG_UL_S_ANG_OFS  (1)
    #define LEG_UL_E_ANG_OFS  (8)

    // Servo Calibration Tables
    #define NUM_TABLE_ELEM 19
    int table_chest_ur[NUM_TABLE_ELEM]    = {1320,1433,1545,1658,1770,1900,2030,2160,2290,2420,2520,2620,2720,2820,2920,3053,3185,3318,3450};
    int table_shoulder_ur[NUM_TABLE_ELEM] = {1380,1480,1605,1710,1830,1960,2065,2195,2315,2440,2545,2655,2770,2875,2985,3110,3210,3330,3440};
    int table_elbow_ur[NUM_TABLE_ELEM]    = {1410,1530,1640,1760,1870,1980,2110,2220,2330,2440,2550,2660,2770,2880,2990,3080,3240,3349,3459};
    int table_chest_br[NUM_TABLE_ELEM]    = {1430,1548,1665,1783,1900,2004,2108,2212,2316,2420,2536,2652,2768,2884,3000,3122,3245,3367,3490};
    int table_shoulder_br[NUM_TABLE_ELEM] = {1380,1500,1620,1720,1850,1980,2105,2210,2325,2440,2540,2665,2800,2900,3030,3150,3255,3360,3475};
    int table_elbow_br[NUM_TABLE_ELEM]    = {1420,1505,1600,1720,1830,1950,2060,2200,2320,2440,2550,2660,2775,2890,3000,3120,3240,3349,3459};
    int table_chest_bl[NUM_TABLE_ELEM]    = {1370,1490,1610,1730,1850,1958,2066,2174,2282,2390,2498,2606,2714,2822,2930,3042,3155,3267,3380};
    int table_shoulder_bl[NUM_TABLE_ELEM] = {1340,1450,1560,1690,1805,1930,2060,2180,2320,2440,2555,2670,2785,2895,3000,3110,3230,3340,3455};
    int table_elbow_bl[NUM_TABLE_ELEM]    = {1387,1498,1614,1750,1855,1980,2110,2210,2330,2440,2550,2690,2795,2920,3020,3150,3260,3380,3480};
    int table_chest_ul[NUM_TABLE_ELEM]    = {1350,1460,1570,1680,1790,1908,2026,2144,2262,2380,2490,2600,2710,2820,2930,3055,3180,3305,3430};
    int table_shoulder_ul[NUM_TABLE_ELEM] = {1370,1480,1600,1715,1840,1950,2080,2190,2320,2440,2560,2680,2790,2900,3010,3140,3245,3345,3460};
    int table_elbow_ul[NUM_TABLE_ELEM]    = {1387,1498,1614,1700,1810,1940,2060,2175,2315,2440,2560,2680,2780,2915,3020,3125,3240,3340,3445};

    // ===================== PARAMETERS =============================================

    #define DEFAULT_LEG_HEIGHT 100 //mm
    #define WIDTH2 60 // half total width
    #define LENGTH2 112.5 // half total length
    #define STEP_HEIGHT 30

    #define MAX_X_ROTATION 30
    #define MAX_Y_ROTATION 30
    #define MAX_Z_ROTATION 30
    #define DEFAULT_BODY_TRANSLATION_SPEED 300
    #define DEFAULT_BODY_ORIENTATION_SPEED 120

    #define DEFAULT_POSITION_oCfF Point(0,0,DEFAULT_LEG_HEIGHT) // Default position relative to the centroid, floor frame

    #define IMU_UPDATE_PERIOD 1 // ms, not used
    #define LEG_UPDATE_PERIOD (3.0/1000) // s

    #define NUM_LEGS 4

    //Timer imu_update_timer;
    Timer leg_update_timer;    
    // ============================= INTERNAL ==================================
    // Legs
    RServoDriver servo_driver;

    DogLeg leg_ur;
    DogLeg leg_ul;
    DogLeg leg_br;
    DogLeg leg_bl;
    DogLeg *foot[NUM_LEGS]; 

    // IMU
    IMU bno_imu;

// ACCESSED
    // ========= Pose/State ============
    // Measured
    Rot   meas_body_orientation_fG2B;   // Actual orientation of the body, relative to stagnant ground frame
    // $ Manually Set
    Rot   set_body_orientation_fF2B;    // Kinematically-set orientation of the body, relative to floor frame.
    Point set_body_position_oCfF;       // Kinematically-set position of the body, FLOOR frame CENTROID origin. Default is DEFAULT_POSITION_oCfF
    // Point Centroid              // Average of PLANTED feet. FLOOR frameFrame::BODY origin, but should convert to GROUND frame during use. 
                                   // Body position = -centroid
    // Experimental
    Point world_centroid_position_oWfF; // Position of the body relative to startup position 
    Rot meas_body_rot_velocity_fG2B;

// PARAMETERS
    // From calibration data
    Point COM_oBfB; // COM relative to default body origin. inFrame::BODY frame


    // ============= COORDINATION =====================
    // Information about a foot to help coordinate body movements: Foot Stance, Anchor Point, Centroid
    // anchor point (fFoC) + centroid (fFoB) = leg position (fFoB)
    FootNote foot_note[NUM_LEGS]; // Coordination information about foot[i] is stored in foot_note[i]

    // ============================Frame::BODY TRAJECTORY ================================
    TrajectoryInfo<Point> body_position_trajectory_oCfF;
    TrajectoryInfo<Rot> body_orientation_trajectory_fF2B;

public:
    // CRITICAL ASSUMPTION: dog starts up in zeroed frames, and all feet at default_height
    // future: need way to auto-calculate all set states/ensure zeroed starting height
    RobotDog() {
        // Set up legs
        foot[0] = &leg_ur;
        foot[1] = &leg_br;
        foot[2] = &leg_bl;
        foot[3] = &leg_ul;

        Point mounting_point;
        Point starting_position = Point(0, 0, DEFAULT_LEG_HEIGHT);
        Point foot_offset = Point(0, 13.97, 0);

        mounting_point = Point( LENGTH2, -WIDTH2, 0);
        leg_ur = DogLeg(&servo_driver,  0,  1,  2, mounting_point, mounting_point - starting_position + -foot_offset); // in future to enforce level starting, find way to set default_position.z = default_height conveniently.
        leg_ur.flipLR();
        leg_ur.setSignalTables(table_chest_ur, table_shoulder_ur, table_elbow_ur);
        leg_ur.calibrateServos(LEG_UR_C_ANG_OFS, LEG_UR_S_ANG_OFS, LEG_UR_E_ANG_OFS);
        leg_ur.setID(0);

        mounting_point = Point(-LENGTH2, -WIDTH2, 0);
        leg_br = DogLeg(&servo_driver,  4,  5,  6, mounting_point, mounting_point - starting_position + -foot_offset);
        leg_br.flipLR();
        leg_br.flipFB();
        leg_br.setSignalTables(table_chest_br, table_shoulder_br, table_elbow_br);
        leg_br.calibrateServos(LEG_BR_C_ANG_OFS, LEG_BR_S_ANG_OFS, LEG_BR_E_ANG_OFS);
        leg_br.setID(1);

        mounting_point = Point(-LENGTH2, WIDTH2, 0);
        leg_bl = DogLeg(&servo_driver,  8,  9, 10, mounting_point, mounting_point - starting_position + foot_offset);
        leg_bl.flipFB();
        leg_bl.setSignalTables(table_chest_bl, table_shoulder_bl, table_elbow_bl);
        leg_bl.calibrateServos(LEG_BL_C_ANG_OFS, LEG_BL_S_ANG_OFS, LEG_BL_E_ANG_OFS);
        leg_bl.setID(2);

        mounting_point = Point( LENGTH2, WIDTH2, 0);
        leg_ul = DogLeg(&servo_driver, 12, 13, 14, mounting_point, mounting_point - starting_position + foot_offset);
        leg_ul.setSignalTables(table_chest_ul, table_shoulder_ul, table_elbow_ul);
        leg_ul.calibrateServos(LEG_UL_C_ANG_OFS, LEG_UL_S_ANG_OFS, LEG_UL_E_ANG_OFS);
        leg_ul.setID(3);

// PARAMETERS
        // COM = Point(3, 0, 9.5);
        COM_oBfB = Point(0,0,0);

// ACCESSED info
        meas_body_orientation_fG2B = ROT_ZERO;   // Dog will start at level height
        set_body_orientation_fF2B = ROT_ZERO;    // Feet default startup will be at normal orientation
        set_body_position_oCfF = starting_position;      // Body will start up at default height 
        meas_body_rot_velocity_fG2B = ROT_ZERO;

        world_centroid_position_oWfF = POINT_ZERO;

        body_position_trajectory_oCfF = TrajectoryInfo<Point>(set_body_position_oCfF);
        body_position_trajectory_oCfF.setSpeed(DEFAULT_BODY_TRANSLATION_SPEED);
        body_orientation_trajectory_fF2B = TrajectoryInfo<Rot>(set_body_orientation_fF2B);
        body_orientation_trajectory_fF2B.setSpeed(DEFAULT_BODY_ORIENTATION_SPEED);

        // Coordination Setup
        for (int i = 0; i < NUM_LEGS; i++) {
            foot_note[i].setAnchorPoint(foot[i]->getDefaultPosition_oBfB() + set_body_position_oCfF);
            foot_note[i].switchStance(FootStance::PLANTED);
        }
    }

    // Startup items to be fulfilled after constructer is called. This must be called to function correctly
    // Sends signals to all legs to go to default heights
    void begin() {
        servo_driver.defaultStartup();

    #ifndef DEBUG_COMPUTATION
        bno_imu.setCollectionMode(IMU::CONTINUOUS);
        bno_imu.defaultStartup();
    #endif

        for (int i = 0; i < NUM_LEGS; i++) {
            foot[i]->moveToPositionFromBodyInstantly(foot[i]->getDefaultPosition_oBfB());
            foot[i]->operate();
        }

        leg_update_timer.usePrecision();
        leg_update_timer.reset(LEG_UPDATE_PERIOD);
        //imu_update_timer.reset(IMU_UPDATE_PERIOD);
    }

    void tareIMU() {
        bno_imu.tareOrientation();
    }

// ================
// ==== HELPER =====
// =================
    Point convertToFrame(Point p, Frame frame_from, Frame frame_to) {
        if (frame_from == frame_to) {
            return p;
        }
        // Convert appropriately
        // use new converter?
        if        ((frame_from == Frame::GROUND) && (frame_to == Frame::BODY)) {
            return p / meas_body_orientation_fG2B;
        } else if ((frame_from == Frame::BODY) && (frame_to == Frame::GROUND)) {
            return p * meas_body_orientation_fG2B;
        } else if ((frame_from == Frame::FLOOR) && (frame_to == Frame::BODY)) {
            return p / set_body_orientation_fF2B;
        } else if ((frame_from == Frame::BODY) && (frame_to == Frame::FLOOR)) {
            return p * set_body_orientation_fF2B;
        } else if ((frame_from == Frame::GROUND) && (frame_to == Frame::FLOOR)) {
            return (p / (meas_body_orientation_fG2B) * set_body_orientation_fF2B);
        } else { // ((frame_from == FLOOR) && (frame_to == GROUND)) {
            return (p / (set_body_orientation_fF2B)) * meas_body_orientation_fG2B;
        } 
    }


// - Convert frame A to B: p_B = p_A / A2B_orientation
// - Convert frame B to A: p_A = p_B * A2B_orientation
    Point convertToFrame(Point p_in_A, Rot A2B_orientation) {
        return p_in_A / A2B_orientation;
    }

    Point convertFromFrame(Point p_in_B, Rot A2B_orientation) {
        return p_in_B * A2B_orientation;
    }

// ========================================
// ====== STATE ACCESSORS =================
// ========================================
    Point getFootPositionFromBody(int foot_i, Frame frame) {
        return convertToFrame(foot[foot_i]->getPosition_oBfB(), Frame::BODY, frame);
    }

    Point getBodyPositionFromCentroid(Frame frame) {
        return convertToFrame(set_body_position_oCfF, Frame::FLOOR, frame);
    }

    Point getCentroidPositionFromBody(Frame frame) {
        return -getBodyPositionFromCentroid(frame);
    }

    Rot getBodyIMUOrientation_fG2B() {
        return meas_body_orientation_fG2B;
    }

    Rot getBodyIMURotVelocity_fG2B() {
        return meas_body_rot_velocity_fG2B;
    }

    Rot getBodyKinematicOrientation_fF2B() {
        return set_body_orientation_fF2B;
    }

    float getStartingHeight() {
        return DEFAULT_LEG_HEIGHT;
    }

    Point getDefaultFootPosition(int foot_i, Frame frame) {
        return convertToFrame(foot[foot_i]->getDefaultPosition_oBfB(), Frame::BODY, frame);
    }

    Point getBodyPositionFromWorld_fF() {
        return world_centroid_position_oWfF;
    }

// ======================================
// ========= COORDINATION ACCESSORS =====
// ======================================
    Point getAnchorPoint_oC(int foot_i, Frame frame=Frame::FLOOR) {
        if (foot_note[foot_i].isAnchored()) {
            return convertToFrame(foot_note[foot_i].getAnchorPoint_oCfF(), Frame::FLOOR, frame);
        } else {
            return POINT_NULL;
        }
    }

    FootStance getFootStance(int foot_i) {
        return foot_note[foot_i].getStance();
    }

    int getFootID(int foot_i) {
        return foot[foot_i]->getID();
    }

// ========================================
// ====== MAIN MOTION COMMANDS/ACCESSORS ============
// ========================================

    void moveBodyToOrientation(Rot goal_orientation) {
        body_orientation_trajectory_fF2B.updateGoal(goal_orientation);
    }

    void moveBodyToPositionFromCentroid(Point goal_position_oC, Frame frame) {
        Point goal_position_oCfF = convertToFrame(goal_position_oC, frame, Frame::FLOOR);
        body_position_trajectory_oCfF.updateGoal(goal_position_oCfF);
    }

    void moveFootToPositionFromBody(int foot_i, Point position_oB, Frame frame) {
        if (foot_note[foot_i].getStance() != FootStance::LIFTED) {
            switchFootStance(foot_i, FootStance::LIFTED);
        }
        Point goal_position_oBfB = convertToFrame(position_oB, frame, Frame::BODY);
        foot[foot_i]->moveToPositionFromBody(goal_position_oBfB);
    }

    void moveBodyToOrientationAtSpeed(Rot goal_orientation, float speed) {
        body_orientation_trajectory_fF2B.setSpeed(speed);
        moveBodyToOrientation(goal_orientation);
    }

    void moveBodyToPositionFromCentroidAtSpeed(Point goal_position_oC, Frame frame, float speed) {
        body_position_trajectory_oCfF.setSpeed(speed);
        moveBodyToPositionFromCentroid(goal_position_oC, frame);
    }

    void moveFootToPositionFromBodyAtSpeed(int foot_i, Point position_oB, Frame frame, float speed) {
        if (foot_note[foot_i].getStance() != FootStance::LIFTED) {
            switchFootStance(foot_i, FootStance::LIFTED);
        }
        Point goal_position_oBfB = convertToFrame(position_oB, frame, Frame::BODY);
        foot[foot_i]->moveToPositionFromBodyAtSpeed(goal_position_oBfB, speed);
    }

    void moveBodyToOrientationInTime(Rot goal_orientation, float time) {
        float speed;
        if (time == TIME_INSTANT) {
            speed = INFINITE_SPEED;
        } else {
            speed = (goal_orientation - set_body_orientation_fF2B).norm()/time; 
        }
        moveBodyToOrientationAtSpeed(goal_orientation, speed);
    }

    void moveBodyToPositionFromCentroidInTime(Point goal_position_oC, Frame frame, float time) {
        Point goal_position_oCfF = convertToFrame(goal_position_oC, frame, Frame::FLOOR);
        float speed;
        if (time == TIME_INSTANT) {
            speed = INFINITE_SPEED;
        } else {
            speed = (goal_position_oCfF - set_body_position_oCfF).norm()/time; 
        }
        moveBodyToPositionFromCentroidAtSpeed(goal_position_oCfF, Frame::FLOOR, speed);
    }

    void moveFootToPositionFromBodyInTime(int foot_i, Point position_oB, Frame frame, float time) {
        if (foot_note[foot_i].getStance() != FootStance::LIFTED) {
            switchFootStance(foot_i, FootStance::LIFTED);
        }
        Point goal_position_oBfB = convertToFrame(position_oB, frame, Frame::BODY);
        foot[foot_i]->moveToPositionFromBodyInTime(goal_position_oBfB, time);
    }

    // Move to original positions and anchors
    void resetDefaultStance() {
        Point starting_position = Point(0, 0, DEFAULT_LEG_HEIGHT);
        for (int i = 0; i < NUM_LEGS; i++) {
            foot[i]->moveToDefaultPosition();
            foot[i]->operate();
        }

        meas_body_orientation_fG2B = ROT_ZERO;   // Dog will start at level height
        set_body_orientation_fF2B = ROT_ZERO;    // Feet default startup will be at normal orientation
        set_body_position_oCfF = starting_position;      // Body will start up at default height 

        world_centroid_position_oWfF = POINT_ZERO;

        // Coordination Setup
        for (int i = 0; i < NUM_LEGS; i++) {
            foot_note[i].setAnchorPoint(foot[i]->getDefaultPosition_oBfB() + set_body_position_oCfF);
        }
    }

// ===============================
// ==== HELPFUL MOTION COMMANDS ====
// ===============================
    /* 
     * @return Calculates the centroid in the FLOOR frame, fromFrame::BODY.
     * However, it is most useful to convert to GROUND frame before use.
     *
     * The centroid is the average of all PLANTED legs.
     * Centroid must be calculated live each time it is used - most robust to changes
     * in anchor points.
     */
    Point calculateCentroid_oBfF() {
        int num_planted = 0;
        for (int i = 0; i < NUM_LEGS; i++) {
            if (foot_note[i].getStance() == FootStance::PLANTED) {
                num_planted++;                
            }
        }

        if (num_planted > 0) {
            Point centroid_oBfB = POINT_ZERO;
            for (int i = 0; i < NUM_LEGS; i++) {
                if (foot_note[i].getStance() == FootStance::PLANTED) {
                    centroid_oBfB += foot[i]->getPosition_oBfB();
                }
            }
            centroid_oBfB /= num_planted;
            return convertFromFrame(centroid_oBfB, set_body_orientation_fF2B);

        } else {
            doError(2);
            return POINT_ZERO;
        }
    }

    // cases
    /*
     * lift <-> plant: centroid, all anchor
     * lift <-> set: anchor
     * plant <-> set: centroid, all anchor
     */
    void switchFootStance(int foot_i, FootStance new_stance) {
        FootStance old_stance = foot_note[foot_i].getStance();

        if (old_stance == new_stance) {
            return;
        }

        if (new_stance == FootStance::LIFTED) {
            // set->lift or plant->lift
            // Nullify the anchor point
            foot_note[foot_i].setAnchorPoint(POINT_NULL);
        }
        if  ((old_stance == FootStance::PLANTED) || (new_stance == FootStance::PLANTED)) {
            Point old_centroid = calculateCentroid_oBfF();
            foot_note[foot_i].switchStance(new_stance);
            Point centroid = calculateCentroid_oBfF();
            set_body_position_oCfF = -centroid;
            
            for (int i = 0; i < NUM_LEGS; i++) {
                if (foot_note[i].isAnchored()) {
                    Point next_anchor = convertFromFrame(foot[i]->getPosition_oBfB(), set_body_orientation_fF2B) + set_body_position_oCfF;
                    foot_note[i].setAnchorPoint(next_anchor);
                }
            }

            Point centroid_motion_fF = centroid - old_centroid;
            world_centroid_position_oWfF += centroid_motion_fF;
            //world_centroid_position_oWfF += convertToFrame(centroid_motion_fF, Frame::FLOOR, Frame::GROUND);
        } else if  (old_stance == FootStance::LIFTED) {
            // lift->set or lift->plant
            Point next_anchor = convertFromFrame(foot[foot_i]->getPosition_oBfB(), set_body_orientation_fF2B) + set_body_position_oCfF;
            foot_note[foot_i].setAnchorPoint(next_anchor);
        }
        foot_note[foot_i].switchStance(new_stance);
    }

// ===============================
// ==== OPERATIONAL COMMANDS ====
// ===============================
    void operate() {
        // Measurements
        #ifndef DEBUG_COMPUTATION // If want to manually feed in orientation data
        bno_imu.operate();
        meas_body_orientation_fG2B = bno_imu.getOrientation();
        meas_body_rot_velocity_fG2B = bno_imu.getRotVelocity();
        #endif

        if (leg_update_timer.timeOut()) {
            leg_update_timer.reset();
            bool anchored_leg_motion_feasible = false;

            if (bodyIsInTrajectory()) {
                Point next_body_position_oCfF = body_position_trajectory_oCfF.getNextState(set_body_position_oCfF);
                Rot next_body_orientation_fF2B = body_orientation_trajectory_fF2B.getNextState(set_body_orientation_fF2B);

                calculateAndSetAnchoredLegMotions(next_body_position_oCfF, next_body_orientation_fF2B);

                anchored_leg_motion_feasible = solveAndCheckAnchoredMotionsFeasible(); 

                if (anchored_leg_motion_feasible) {
                    set_body_orientation_fF2B = next_body_orientation_fF2B;
                    set_body_position_oCfF = next_body_position_oCfF;
                } else { 
                    body_orientation_trajectory_fF2B.end();
                    body_position_trajectory_oCfF.end();
                }
            } else {
                syncTrajectoryTimers();
            }

            solveLiftedLegMotions();

            checkAndActuateLiftedLegMotion();

            if(anchored_leg_motion_feasible) {
                actuateAnchoredLegMotions();
            }
        }


    }

    bool bodyIsInTrajectory() {
        return (body_position_trajectory_oCfF.isInMotion() || body_orientation_trajectory_fF2B.isInMotion());
    }

    void calculateAndSetAnchoredLegMotions(Point next_body_position_oCfF, Rot next_body_orientation_fF2B) {
        for (int i = 0; i < NUM_LEGS; i++) {
            if (foot_note[i].isAnchored()) {
                // use the new converter
                foot[i]->moveToPositionFromBodyInstantly((foot_note[i].getAnchorPoint_oCfF() - next_body_position_oCfF) / next_body_orientation_fF2B);
            }
        }
    }

    bool solveAndCheckAnchoredMotionsFeasible() {
        for (int i = 0; i < NUM_LEGS; i++) {
            if (foot_note[i].isAnchored()) {
                foot[i]->solveMotion();
                if(!(foot[i]->kinematicsIsValid())) { // Failure detected
                    return false;
                }
            }
        }
        return true;
    }

    void solveLiftedLegMotions() {
        for (int i = 0; i < NUM_LEGS; i++) {
            if ((foot_note[i].getStance() == FootStance::LIFTED)) {
                foot[i]->solveMotion();
            }
        }
    }

    void checkAndActuateLiftedLegMotion() {
        for (int i = 0; i < NUM_LEGS; i++) {
            if (foot_note[i].getStance() == FootStance::LIFTED) {
                if (foot[i]->kinematicsIsValid()) {
                    foot[i]->sendSignalsAndSavePosition();
                } else {
                    foot[i]->endTrajectory();
                }
            } 
        }
    }

    void actuateAnchoredLegMotions() {
        for (int i = 0; i < NUM_LEGS; i++) {
            if (foot_note[i].isAnchored())
                    foot[i]->sendSignalsAndSavePosition();
        }
    }

    void syncTrajectoryTimers() {
        body_position_trajectory_oCfF.syncTimer();
        body_orientation_trajectory_fF2B.syncTimer();
    }

// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ======== DEBUGGING =====================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
public:
    void doError(int e) {
        #ifdef DEBUG
        Serial.print("Dog ERROR: ");
        switch (e) {
            case 1: Serial.println("Leg IKIN invalid."); break;
            case 2: Serial.println("No PLANTED Points."); break;
            case 3: Serial.println("Some FootNote Error"); break;
        } 
        #endif
    }

    #ifdef DEBUG
    DogLeg *getLeg(int i) {
        return foot[i];
    }

    RServoDriver *getServoDriver() {
        return &servo_driver;
    }
    #endif

    #ifdef DEBUG_COMPUTATION
    // Manually feed in orientation values
    void feedIMU(Rot orientation) {
        meas_body_orientation_fG2B = orientation;
    }


    // void printIMU() {
    //     Serial.print("IMU: "); (bno_imu.getOrientation()).print();
    // }

    // void printLegs() {
    //     for (int i = 0; i < NUM_LEGS; i++) {
    //         Point foot_pos_G = foot[i]->getCurrentFootPositionFromShoulder(Frame::GROUND);
    //         Serial.print("Leg "); Serial.print(i); Serial.print(": "); foot_pos_G.print();
    //     }
    // }

    // void printLegsFromBody() {
    //     for (int i = 0; i < NUM_LEGS; i++) {
    //         Point foot_pos_G = foot[i]->getCurrentFootPositionFromBody(Frame::GROUND);
    //         Serial.print("Leg "); Serial.print(i); Serial.print(": "); foot_pos_G.print();
    //     }
    // }

    // void printLeg(int i) {
    //     Point foot_pos_G = foot[i]->getCurrentFootPositionFromShoulder(Frame::GROUND);
    //     Serial.print("Leg "); Serial.print(i); Serial.print(": "); foot_pos_G.print();
    // }
    #endif

};


#endif