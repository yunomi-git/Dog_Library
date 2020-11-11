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

// #define DEBUG

// TODO: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// - implement better way to send signals at the same time...not all necessary checks are occuring
// - clean up code, fix variable names
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
// ORIENTATION
// - All Leg commands are sent in body frame. Dog class responsible for correct frame conversion.
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



enum class Frame {GROUND, BODY, FLOOR};
enum class FootStance {PLANTED, SET, LIFTED};

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
                                   // Body position is measured relative to the centroid (in the opposite direction).
                                   // The centroid is 
    // Experimental
    Point world_centroid_position_oWfG; // Position of the body relative to startup position 

// PARAMETERS
    // From calibration data
    Point COM_oBfB; // COM relative to default body origin. inFrame::BODY frame


    // ============= COORDINATION =====================
    // Foot Stance, Foot Note, Anchor Point, Centroid

    // Information about a foot to help coordinate body movements
    // anchor point (fFoC) + centroid (fFoB) = leg position (fFoB)
    struct FootNote {
    private:
        FootStance stance;
        Point anchor_point_oCfF; // FLOOR frameFrame::BODY origin. The position at which a foot touches the ground
    
    public:
        FootNote() = default;

        FootNote(Point new_anchor_point_oCfF) {
            stance = FootStance::PLANTED;
            anchor_point_oCfF = new_anchor_point_oCfF;
        }

        void setAnchorPoint(Point anchor) {
            anchor_point_oCfF = anchor;
        }

        void switchStance(FootStance new_stance) {
            if (new_stance == stance) {
                return;
            }

            stance = new_stance;
        }

        Point getAnchorPoint_oCfF() {
            if (stance == FootStance::LIFTED) {
                return POINT_NULL;
            }
            return anchor_point_oCfF;
        }

        FootStance getStance() {
            return stance;
        }

        bool isAnchored() {
            return !(stance == FootStance::LIFTED);
        }
    };

// ACCESSED
    FootNote foot_note[NUM_LEGS]; // Coordination information about foot[i] is stored in foot_note[i]

    /* 
     * @return Calculates the centroid in the FLOOR frame, fromFrame::BODY.
     * However, it is most useful to convert to GROUND frame before use.
     *
     * The centroid is the average of all PLANTED legs.
     * Centroid must be calculated live each time it is used - most robust to changes
     * in anchor points.
     */
    Point getCentroid_oBfF() {
        // 1. average
        int num_planted = 0;
        Point centroid_oBfB = POINT_ZERO;
        for (int i = 0; i < NUM_LEGS; i++) {
            if (foot_note[i].getStance() == FootStance::PLANTED) {
                centroid_oBfB += foot[i]->getPosition_oBfB();
                num_planted++;                
            }
        }
        
        if (num_planted == 0) {
            doError(2);
            return POINT_ZERO;
        } 

        centroid_oBfB /= num_planted;
        return centroid_oBfB / set_body_orientation_fF2B;
    }

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
        //leg_ur.setSignalTables(table_chest_ur, table_shoulder_ur, table_elbow_ur);
        leg_ur.calibrateServos(LEG_UR_C_ANG_OFS, LEG_UR_S_ANG_OFS, LEG_UR_E_ANG_OFS);
        leg_ur.setID(0);

        mounting_point = Point(-LENGTH2, -WIDTH2, 0);
        leg_br = DogLeg(&servo_driver,  4,  5,  6, mounting_point, mounting_point - starting_position + -foot_offset);
        leg_br.flipLR();
        leg_br.flipFB();
        //leg_br.setSignalTables(table_chest_br, table_shoulder_br, table_elbow_br);
        leg_br.calibrateServos(LEG_BR_C_ANG_OFS, LEG_BR_S_ANG_OFS, LEG_BR_E_ANG_OFS);
        leg_br.setID(1);

        mounting_point = Point(-LENGTH2, WIDTH2, 0);
        leg_bl = DogLeg(&servo_driver,  8,  9, 10, mounting_point, mounting_point - starting_position + foot_offset);
        leg_bl.flipFB();
        //leg_bl.setSignalTables(table_chest_bl, table_shoulder_bl, table_elbow_bl);
        leg_bl.calibrateServos(LEG_BL_C_ANG_OFS, LEG_BL_S_ANG_OFS, LEG_BL_E_ANG_OFS);
        leg_bl.setID(2);

        mounting_point = Point( LENGTH2, WIDTH2, 0);
        leg_ul = DogLeg(&servo_driver, 12, 13, 14, mounting_point, mounting_point - starting_position + foot_offset);
        //leg_ul.setSignalTables(table_chest_ul, table_shoulder_ul, table_elbow_ul);
        leg_ul.calibrateServos(LEG_UL_C_ANG_OFS, LEG_UL_S_ANG_OFS, LEG_UL_E_ANG_OFS);
        leg_ul.setID(3);


// PARAMETERS
        // COM = Point(3, 0, 9.5);
        COM_oBfB = Point(0,0,0);

// ACCESSED info
        meas_body_orientation_fG2B = ROT_ZERO;   // Dog will start at level height
        set_body_orientation_fF2B = ROT_ZERO;    // Feet default startup will be at normal orientation
        set_body_position_oCfF = starting_position;      // Body will start up at default height 

        world_centroid_position_oWfG = POINT_ZERO;

        // Coordination Setup
        for (int i = 0; i < NUM_LEGS; i++) {
            foot_note[i].setAnchorPoint(foot[i]->getDefaultPosition_oBfB() + set_body_position_oCfF);
        }
    }

    // Startup items to be fulfilled after constructer is called. This must be called to function correctly
    // Sends signals to all legs to go to default heights
    void begin() {
        servo_driver.defaultStartup();

    #ifndef DEBUG_COMPUTATION
        bno_imu.setCollectionMode(IMU::SINGLE);
        bno_imu.defaultStartup();
    #endif

        for (int i = 0; i < NUM_LEGS; i++) {
            foot[i]->moveToPositionFromBody(foot[i]->getDefaultPosition_oBfB());
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
// ========================================
// ====== STATE ACCESSORS =================
// ========================================
    Point getFootPositionFromBody(int foot_i, Frame frame) {
        return convertToFrame(foot[foot_i]->getPosition_oBfB(), Frame::BODY, frame);
    }

    Point getBodyPositionFromCentroid(Frame frame) {
        return convertToFrame(set_body_position_oCfF, Frame::FLOOR, frame);
    }

    Rot getBodyIMUOrientation_fG2B() {
        return meas_body_orientation_fG2B;
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




// ========================================
// ====== MAIN MOTION COMMANDS/ACCESSORS ============
// ========================================
// Commands
    // Orientation Only
    // moves body relative to floor frame. to move relative to ground frame, manually calculate beforehand
    void moveBodyToOrientation(Rot goal_orientation, float time=TIME_INSTANT) {
        body_orientation_trajectory_fF2B.begin(goal_orientation, time);
    }

    // Position Only
    void moveBodyToPositionFromCentroid(Point goal_position, Frame frame, float time=TIME_INSTANT) {
        body_position_trajectory_oCfF.begin(convertToFrame(goal_position, frame, Frame::FLOOR), time);
    }

    // Do both at the same time
    void moveBodyToPose(Rot goal_orientation, Point goal_position, Frame frame, float time=TIME_INSTANT) {
        body_orientation_trajectory_fF2B.begin(goal_orientation, time);
        body_position_trajectory_oCfF.begin(convertToFrame(goal_position, frame, Frame::FLOOR), time);
    }
    
    void moveFootToPositionFromBody(int foot_i, Point position, Frame frame, float time=TIME_INSTANT) {
        if (foot_note[foot_i].getStance() != FootStance::LIFTED) {
            switchFootStance(foot_i, FootStance::LIFTED);
        }
        // set up the trajectory
        foot[foot_i]->moveToPositionFromBody(convertToFrame(position, frame, Frame::BODY), time);
    }

// Accessors


// SPECIAL MOTION COMMANDS

    void adjustLegMotionGoal(int foot_i, Point new_foot_pos) {
        foot[foot_i]->adjustLegMotionGoal(new_foot_pos);
    }

    void adjustLegMotionTime(int foot_i, float time) {
        foot[foot_i]->adjustLegMotionTime(time);
    }

    void adjustBodyPositionMotionTime(float time) {
        body_position_trajectory_oCfF.adjustFinalTime(time);
    }

    void adjustBodyOrientationMotionTime(float time) {
        body_orientation_trajectory_fF2B.adjustFinalTime(time);
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

        world_centroid_position_oWfG = POINT_ZERO;

        // Coordination Setup
        for (int i = 0; i < NUM_LEGS; i++) {
            foot_note[i].setAnchorPoint(foot[i]->getDefaultPosition_oBfB() + set_body_position_oCfF);
        }
    }

    void cancelTrajectory() {
        // cancels all body-based motion
    }

    void stopMotion() {
        // cancels all motion whatsoever
    }
// Accessors



// ===============================
// ==== HELPFUL MOTION COMMANDS ====
// ===============================
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
            //Point old_centroid = getCentroid_oBfF();
            foot_note[foot_i].switchStance(new_stance);
            Point centroid = getCentroid_oBfF();
            set_body_position_oCfF = -centroid;
            
            for (int i = 0; i < NUM_LEGS; i++) {
                if (foot_note[i].isAnchored())
                    foot_note[i].setAnchorPoint(foot[i]->getPosition_oBfB() * set_body_orientation_fF2B + set_body_position_oCfF);
            }

            //Point centroid_motion_fF = centroid - old_centroid;
            //world_centroid_position_oWfG += convertToFrame(centroid_motion_fF, Frame::FLOOR, Frame::GROUND);
        } else if  (old_stance == FootStance::LIFTED) {
            // lift->set or lift->plant
            foot_note[foot_i].setAnchorPoint(foot[foot_i]->getPosition_oBfB() * set_body_orientation_fF2B + set_body_position_oCfF);
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
        #endif

        if (leg_update_timer.timeOut()) {
            leg_update_timer.reset();
            bool anchored_leg_motion_feasible = true;
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // ~~~~ Solves body motion (for anchored legs) ~~~~
            if (body_position_trajectory_oCfF.isActive() || body_orientation_trajectory_fF2B.isActive()) {
                // Updates the next pose on the trajectory
                Point next_body_position_oCfF = body_position_trajectory_oCfF.getNextState(set_body_position_oCfF);
                Rot next_body_orientation_fF2B = body_orientation_trajectory_fF2B.getNextState(set_body_orientation_fF2B);

                // then solves IKIN for all anchored legs
                for (int i = 0; i < NUM_LEGS; i++) {
                    if (foot_note[i].isAnchored()) {
                        foot[i]->moveToPositionFromBody((foot_note[i].getAnchorPoint_oCfF() - next_body_position_oCfF) / next_body_orientation_fF2B, TIME_INSTANT); // TODO: Check that this conversion is correct
                    }
                }

                // Do kinematic checks and sends the signals
                // First checks if anchored legs motions are possible
                for (int i = 0; i < NUM_LEGS; i++) {
                    if (foot_note[i].isAnchored()) {
                        foot[i]->solveMotion();
                        if(!(foot[i]->kinematicsIsValid())) { // Failure detected
                            anchored_leg_motion_feasible = false;
                            break;
                        }
                    }
                }

                // If so, saves values
                if (anchored_leg_motion_feasible) {
                    set_body_orientation_fF2B = next_body_orientation_fF2B;
                    set_body_position_oCfF = next_body_position_oCfF;
                } else { // Otherwise, cancels the body trajectories
                    body_orientation_trajectory_fF2B.end();
                    body_position_trajectory_oCfF.end();
                    // // Feet should not have trajectories, but adds redundancy
                    // for (int i = 0; i < NUM_LEGS; i++) {
                    //     if (foot_note[i].isAnchored()) {
                    //         foot[i]->endTrajectory();
                    //     } 
                    // }
                }
            }



            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // ~~~~ Solves individual leg motion (for lifted legs) ~~~~
            for (int i = 0; i < NUM_LEGS; i++) {
                if ((foot_note[i].getStance() == FootStance::LIFTED)) {
                    foot[i]->solveMotion();
                }
            }

            // ~~~~~~~~~~~~~~~~~~
            // ~~ Send the signals ~~~
            for (int i = 0; i < NUM_LEGS; i++) {
                if (foot_note[i].isAnchored() && (body_position_trajectory_oCfF.isActive() || body_orientation_trajectory_fF2B.isActive())) {
                    if (anchored_leg_motion_feasible)
                        foot[i]->sendSignals();
                } else { // foot is lifted
                    if (foot[i]->kinematicsIsValid()) {
                        foot[i]->sendSignals();
                    } else {
                        foot[i]->endTrajectory();
                    }
                } 
            }
        }


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
    // bool fixedLegPresent() {
    //     for (int i = 0; i < NUM_LEGS; i++) {
    //         if (foot[i]->isFixed()) {
    //             return true;
    //         }
    //     }
    //     return false;
    // }

    // bool isIdle() {
    //     for (int i = 0; i < NUM_LEGS; i++) {
    //         if (!(foot[i]->isIdle()))
    //             return false;
    //     }
    //     return true;
    // }


    //     // "heights" are relative to body origin
    // void gotoLegHeights(float h1, float h2, float h3, float h4) {
    //     foot[0]->setToPositionFromShoulder(Point(0, 0, h1));
    //     foot[1]->setToPositionFromShoulder(Point(0, 0, h2));
    //     foot[2]->setToPositionFromShoulder(Point(0, 0, h3));
    //     foot[3]->setToPositionFromShoulder(Point(0, 0, h4));
    //     operateAllLegs();
    //     sendAllSignals();          
    // }


    // void sendAllSignals() {
    //     for (int i = 0; i < NUM_LEGS; i++) {
    //         foot[i]->sendSignals();
    //     }
    // }



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

    // void printCentroid() {
    //     Serial.print("Centoid (oBfG): "); centroid_oBfG.print();
    // }


#endif
};


#endif


    // // Coordinates based on centroid of PLANTED feet
    // // Centroid is the centroid of the polygon created by the PLANTED feet, at the default height
    // // Kinematically moves dog body to a new orientation/position relative to current pose.
    // // New orientation may not be exactly equivalent to measured orientation.
    // // Only PLANTED/FIXED legs get moved.
    // // Implementation: 
    // // @param r:                  Desired body orientation
    // // @param p_oC:               Desired body position, relative to centroid
    // // @param frame:              Reference frame in which dp is given. IfFrame::BODY is chosen, movement based on *DESIRED* body orientation.
    // // @param time:               Total movement time desired to move the foot from its current position to the new position.
    // //                            Set to 0/TIME_INSTANT for max speed.
    // void setFromCentroid(Rot r, Point p_oCf, Frame frame, float time=TIME_INSTANT) {
    //     // Calculate distance to move, in GROUND frame
    //     Point dp_fB;

    //     if (frame == Frame::GROUND) {
    //         dp_fB = (p_oCf + centroid_oBfG) * r;
    //     } else { //Frame::BODY frame
    //         dp_fB = p_oCf + centroid_oBfG * r;
    //     }

    //     desired_orientation = r;

    //     // Calculates new positions
    //     Point new_leg_position_oBfB;
    //     for (int i = 0; i < NUM_LEGS; i++) {
    //         if (foot[i]->isPLANTED()) {
    //             // Serial.print("Original (b): "); (foot[i]->getCurrentFootPositionFromBody(Frame::BODY)).print();
    //             // Serial.print("Original (g): "); (foot[i]->getCurrentFootPositionFromBody(Frame::GROUND)).print();
    //             new_leg_position_oBfB = foot[i]->getCurrentFootPositionFromBody(Frame::GROUND) * r; // Rotates to desired orientation
    //             new_leg_position_oBfB -= dp_fB; // Translate body accordingly
    //             // Serial.print("Goal (b): "); new_leg_position_oBfB.print();
    //             // Serial.print("Goal (g): "); (new_leg_position_oBfB / r).print();
    //             foot[i]->setToPositionFromBody(new_leg_position_oBfB, Frame::BODY, time);
    //         }
    //     }
    //     // Serial.println();
    // }


// Non-Trajectory Walking
    // Body moves by gait length.
    // Leg moves by 2x gait length
    // FUTURE: must account for if leg does not start on ground..."current height"?
//     void step(float direction_yaw, float gait_length) {
//         if (gait_state == 1) {
//             foot[0]->setPLANTED(true);
//             foot[1]->setPLANTED(false);
//             foot[2]->setPLANTED(false);
//             foot[3]->setPLANTED(true);
//         } else {
//             foot[0]->setPLANTED(false);
//             foot[1]->setPLANTED(true);
//             foot[2]->setPLANTED(true);
//             foot[3]->setPLANTED(false);
//         }
//         // Raise moving legs
//         for (int i = 0; i < NUM_LEGS; i++) {
//             if (!(foot[i]->isPLANTED())) {
//               foot[i]->setToPositionFromShoulder(Point(gait_length * cos(direction_yaw DEG), 
//                                                       gait_length * sin(direction_yaw DEG), 
//                                                       -height + STEP_HEIGHT), 
//                                                 Frame::GROUND);
//               foot[i]->sendSignals();
//             }
//         }
//         delay(50);

//         // Tilt body forward/Push PLANTED legs
//         moveByPosition(gait_length * cos(direction_yaw DEG), gait_length * sin(direction_yaw DEG), 0);

//         delay(50);
//         // Put down moving legs
//         for (int i = 0; i < NUM_LEGS; i++) {
//             if (!(foot[i]->isPLANTED())) {
//                 foot[i]->gotoPositionFromShoulder(Point(2*gait_length * cos(direction_yaw DEG), 
//                                                        2*gait_length * sin(direction_yaw DEG), 
//                                                        -height), 
//                                                  Frame::GROUND);
//                 foot[i]->sendSignals();
//             }

//         }
//         gait_state *= -1;
//     }

//     void wave_step(float direction_yaw, float gait_length) {
//     // Serial.println("wave");
//         for (int i = 0; i < NUM_LEGS; i++) {
//             if (i == wave_state) {
//                 foot[i]->setPLANTED(false);
//             } else {
//                 foot[i]->setPLANTED(true);
//             }
//         }
// //         Serial.print("Stepping ");Serial.println(wave_state);
// // printLegs();
//  // Serial.println("Raising Leg");
//         // Raise moving legs
//         for (int i = 0; i < NUM_LEGS; i++) {
//              Serial.println(i);
//             if (!(foot[i]->isPLANTED())) {
//               // Serial.println("ey");
//               // foot[i]->gotoPositionFromShoulder(Point(gait_length * cos(direction_yaw DEG), gait_length * sin(direction_yaw DEG), -height + STEP_HEIGHT));
//               foot[i]->moveByPosition(Point(1.5*gait_length * cos(direction_yaw DEG), 1.5*gait_length * sin(direction_yaw DEG), STEP_HEIGHT), orientation);
//               foot[i]->sendSignals();
//             }
//             // printLegs();
//         }
//           // Serial.println("Leg Raised");

//         delay(25);
// // printLegs();
//           // Tilt body forward/Push PLANTED legs
//         moveByPosition(gait_length * cos(direction_yaw DEG), gait_length * sin(direction_yaw DEG), 0);

//         // delay(25);
//         // Put down moving legs
//         for (int i = 0; i < NUM_LEGS; i++) {
//             if (!(foot[i]->isPLANTED())) {
//                 foot[i]->moveByPosition(Point(1.5*gait_length * cos(direction_yaw DEG), 1.5*gait_length * sin(direction_yaw DEG), -STEP_HEIGHT), orientation);
//               // foot[i]->gotoPositonFromBody(Point(2*gait_length * cos(direction_yaw DEG), 2*gait_length * sin(direction_yaw DEG), -STEP_HEIGHT), orientation);
//                 foot[i]->sendSignals();
//             }
//         }
//         wave_state = (wave_state + 1)%NUM_LEGS;
//     }