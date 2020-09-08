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

// #define DEBUG

// TODO: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// - Make sure everything is oBfG
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// USAGE DETAILS
// Nomenclature:
//  - Foot is used interchangeably with Leg. Foot is generally used to refer to the contact point.
//    Leg is used to refer to motions.
// FOOT STANCE
// - Foot may be set to various stances:
//   Planted: Foot trajectory is constant in ground frame regardless of body orientation
//   Floating: Foot trajectory is constant in body frame
//   Set: Foot position is constant in ground frame, and foot is in contact with the ground/a surface
// - For consistency with coordination states, requesting a movement in a given frame...creates
//   a trajectory dependent on the coordination state
//
// ORIENTATION // rewrite this
// - The body orientation refers to the CURRENT orientation, not the desired one.
//   It is primarily used for - accessing the current foot position
//                            - locating the ground frame
//   Obtaining a desired orientation is the responsibility of the dog class.

/**
* IMPLEMENTATION DETAILS
* - All points are in GROUND FRAME and RELATIVE TO BODY ORIGIN unless otherwise noted
* 
* - Convert ground to body frame: p_B = p_G * orientation
* - Convert body to ground frame: p_G = p_B / (orientation)
*
*/



enum class Frame {GROUND, BODY};

class RobotDog {
    // ======================= CALIBRATION ===============================================
    #define LEG_UR_C_ANG_OFS  (0)
    #define LEG_UR_S_ANG_OFS  (-3)
    #define LEG_UR_E_ANG_OFS  (6)

    #define LEG_BR_C_ANG_OFS  (0)
    #define LEG_BR_S_ANG_OFS  (3.5)
    #define LEG_BR_E_ANG_OFS  (1)

    #define LEG_BL_C_ANG_OFS  (0)
    #define LEG_BL_S_ANG_OFS  (1)
    #define LEG_BL_E_ANG_OFS  (-5)

    #define LEG_UL_C_ANG_OFS  (0)
    #define LEG_UL_S_ANG_OFS  (-1)
    #define LEG_UL_E_ANG_OFS  (-8)

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

    #define IMU_UPDATE_PERIOD 1 // ms
    #define LEG_UPDATE_PERIOD 3 // ms

    #define NUM_LEGS 4

    Timer imu_update_timer;
    Timer leg_update_timer;     // in 
    // ============================= INTERNAL ==================================
    // Legs
    RServoDriver servo_driver;

    DogLeg leg_ur;
    DogLeg leg_ul;
    DogLeg leg_br;
    DogLeg leg_bl;

    DogLeg *leg[NUM_LEGS]; 


    // IMU
    IMU bno_imu;

    // Pose/State
    Point centroid_oBfG;        // Centroid location in GROUND frame relative to BODY origin
    Rot body_orientation;       // Actual orientation of the body, relative to stagnant ground frame
    Rot desired_orientation;    // Desired orientation of the body
    Point COM; // COM relative to default body origin

    // ============= COORDINATION =====================
    enum class FootStance {PLANTED, SET, LIFTED};

    // Information about a foot to help coordinate body movements
    struct FootNote {
    private:
        FootStance stance;
        Point anchor_point; // GROUND frame BODY origin. The position at which a foot touches the ground
    
    public:
        FootNote() = default;

        FootNote(Point nanchor_point) {
            stance = FootStance::PLANTED;
            anchor_point = nanchor_point;
        }

        void switchStance(FootStance new_stance, Point new_anchor_point=POINT_ZERO) {
            if (new_stance == stance) {
                return;
            }
            // Setting foot onto floor from lifted: needs a new anchor point defined
            if ((new_stance != FootStance::LIFTED) && (stance == FootStance::LIFTED)) {
                if (nanchor_point == POINT_ZERO) {
                    doError(3);
                    return;
                } else {
                    anchor_point = new_anchor_point;
                }
            // Lifting foor from the floor. Just prints a warning if attempted to set an anchor point
            } else if ((new_stance == FootStance::LIFTED)) {
                if (new_anchor_point != POINT_ZERO) {
                    doError(3);
                }
            }

            stance = new_stance;
        }

        Point getAnchorPoint() {
            if (stance == FootStance::FREE) {
                doError(3);
                return POINT_ZERO;
            }
            return anchor_point;
        }

        FootStance getStance() {
            return stance;
        }
    }

    FootNote[NUM_LEGS] foot_note;


public:
    RobotDog() {
        // Set up legs
        leg[0] = &leg_ur;
        leg[1] = &leg_br;
        leg[2] = &leg_bl;
        leg[3] = &leg_ul;

        // COM = Point(3, 0, 9.5);
        COM = Point(0,0,0);

        leg_ur = DogLeg(&servo_driver,  0,  1,  2, false, false, Point( LENGTH2,  WIDTH2, 0) - COM, DEFAULT_LEG_HEIGHT);
        leg_ur.setSignalTables(table_chest_ur, table_shoulder_ur, table_elbow_ur);
        leg_ur.calibrateServos(LEG_UR_C_ANG_OFS, LEG_UR_S_ANG_OFS, LEG_UR_E_ANG_OFS);

        leg_br = DogLeg(&servo_driver,  4,  5,  6, false,  true, Point(-LENGTH2,  WIDTH2, 0) - COM, DEFAULT_LEG_HEIGHT);
        leg_br.setSignalTables(table_chest_br, table_shoulder_br, table_elbow_br);
        leg_br.calibrateServos(LEG_BR_C_ANG_OFS, LEG_BR_S_ANG_OFS, LEG_BR_E_ANG_OFS);

        leg_bl = DogLeg(&servo_driver,  8,  9, 10,  true,  true, Point(-LENGTH2, -WIDTH2, 0) - COM, DEFAULT_LEG_HEIGHT);
        leg_bl.setSignalTables(table_chest_bl, table_shoulder_bl, table_elbow_bl);
        leg_bl.calibrateServos(LEG_BL_C_ANG_OFS, LEG_BL_S_ANG_OFS, LEG_BL_E_ANG_OFS);

        leg_ul = DogLeg(&servo_driver, 12, 13, 14,  true, false, Point( LENGTH2, -WIDTH2, 0) - COM, DEFAULT_LEG_HEIGHT);
        leg_ul.setSignalTables(table_chest_ul, table_shoulder_ul, table_elbow_ul);
        leg_ul.calibrateServos(LEG_UL_C_ANG_OFS, LEG_UL_S_ANG_OFS, LEG_UL_E_ANG_OFS);

        // Coordination Setup
        for (int i = 0; i < NUM_LEGS; i++) {
            foot_note[i] = FootNote(leg[i].getMountingPoint() + Point(0, 0, -DEFAULT_LEG_HEIGHT));
        }

        // Other setup
        desired_orientation = ROT_ZERO;
    }

    void begin() {
        servo_driver.defaultStartup();

        bno_imu.setCollectionMode(IMU::CONTINUOUS);
        bno_imu.defaultStartup();

        leg_update_timer.reset(LEG_UPDATE_PERIOD);
        imu_update_timer.reset(IMU_UPDATE_PERIOD);
    }

    void tareIMU() {
        bno_imu.tareOrientation();
    }

    // "heights" are relative to body origin
    void gotoLegHeights(float h1, float h2, float h3, float h4) {
        leg[0]->setToPositionFromShoulder(Point(0, 0, h1), Frame::BODY);
        leg[1]->setToPositionFromShoulder(Point(0, 0, h2), Frame::BODY);
        leg[2]->setToPositionFromShoulder(Point(0, 0, h3), Frame::BODY);
        leg[3]->setToPositionFromShoulder(Point(0, 0, h4), Frame::BODY);
        operateAllLegs();
        sendAllSignals();          
    }



    // Kinematically moves dog body to a new orientation/position relative to current pose.
    // New orientation may not be exactly equivalent to measured orientation.
    // Only PLANTED/FIXED legs get moved.
    // Implementation: 
    // @param d_orientation:      Change in body orientation
    // @param d_position:         Change in body position, in requested frame
    // @param frame:              Reference frame in which dp is given. If BODY is chosen, movement based on *DESIRED* body orientation.
    // @param time:               Total movement time desired to move the foot from its current position to the new position.
    //                            Set to 0/TIME_INSTANT for max speed.
    void setFromPosition(Rot dr, Point dp, Frame frame, float time=TIME_INSTANT) {
        Rot r = body_orientation + dr; // Desired orientation
        desired_orientation = r;
        // Convert everything to GROUND frame
        if (frame == Frame::BODY) {
            dp /= r;
        }

        // Calculates new positions
        Point new_leg_position_oBfG;
        for (int i = 0; i < NUM_LEGS; i++) {
            if (foot_note[i].getStance() != FootStance::LIFTED) {
                new_leg_position_oBfG = leg[i]->getCurrentFootPositionFromBody(Frame::GROUND) * r; // Rotate body to the desired orientation
                new_leg_position_oBfG -= dp; // Translate the body accordingly
                leg[i]->setToPositionFromBody(new_leg_position_oBfG, Frame::GROUND, time);
            }
        }
    }

    // Coordinates based on centroid of PLANTED feet
    // Centroid is the centroid of the polygon created by the PLANTED feet, at the default height
    // Kinematically moves dog body to a new orientation/position relative to current pose.
    // New orientation may not be exactly equivalent to measured orientation.
    // Only PLANTED/FIXED legs get moved.
    // Implementation: 
    // @param r:                  Desired body orientation
    // @param p_oC:               Desired body position, relative to centroid
    // @param frame:              Reference frame in which dp is given. If BODY is chosen, movement based on *DESIRED* body orientation.
    // @param time:               Total movement time desired to move the foot from its current position to the new position.
    //                            Set to 0/TIME_INSTANT for max speed.
    void setFromCentroid(Rot r, Point p_oCf, Frame frame, float time=TIME_INSTANT) {
        // Calculate distance to move, in GROUND frame
        Point dp_fB;

        if (frame == Frame::GROUND) {
            dp_fB = (p_oCf + centroid_oBfG) * r;
        } else { // BODY frame
            dp_fB = p_oCf + centroid_oBfG * r;
        }

        desired_orientation = r;

        // Calculates new positions
        Point new_leg_position_oBfB;
        for (int i = 0; i < NUM_LEGS; i++) {
            if (leg[i]->isPLANTED()) {
                // Serial.print("Original (b): "); (leg[i]->getCurrentFootPositionFromBody(Frame::BODY)).print();
                // Serial.print("Original (g): "); (leg[i]->getCurrentFootPositionFromBody(Frame::GROUND)).print();
                new_leg_position_oBfB = leg[i]->getCurrentFootPositionFromBody(Frame::GROUND) * r; // Rotates to desired orientation
                new_leg_position_oBfB -= dp_fB; // Translate body accordingly
                // Serial.print("Goal (b): "); new_leg_position_oBfB.print();
                // Serial.print("Goal (g): "); (new_leg_position_oBfB / r).print();
                leg[i]->setToPositionFromBody(new_leg_position_oBfB, Frame::BODY, time);
            }
        }
        // Serial.println();
    }

    // Calculates the centroid in the GROUND frame, from BODY
    // Only use as needed - the centroid does not change frequently
    Point calculateCentroid() {
        int num_PLANTED = 0;
        float lowest_leg_height = 0;
        Point centroid = POINT_ZERO;
        for (int i = 0; i < NUM_LEGS; i++) {
            if (foot_note[i].getStance == FootStance::PLANTED) {
                Point leg_pos_oBfG = foot_note[i].getAnchorPoint();
                centroid += leg_pos_oBfG; // get x, y of centroid

                if (leg_pos_oBfG.z < lowest_leg_height)
                    lowest_leg_height = leg_pos_oBfG.z;

                num_PLANTED++;                
            }
        }
        
        if (num_PLANTED == 0) {
            doError(2);
            return POINT_ZERO;
        } 

        centroid /= num_PLANTED;
        centroid.z = lowest_leg_height; // Calculate actual centroid
        
        return centroid;
    }


    // Move to original positions and anchors
    void gotoDefaultStance() {
        for (int i = 0; i < NUM_LEGS; i++) {
            leg[i]->setToPositionFromShoulder(Point(0, 0, -DEFAULT_LEG_HEIGHT));
            leg[i]->setstance(Footstance::PLANTED);
        }
        operateAllLegs();
        sendAllSignals();
    }

    void sendAllSignals() {
        for (int i = 0; i < NUM_LEGS; i++) {
            leg[i]->sendSignals();
        }
    }

    void operate() {
        // Update the IMU and centroid
        if (imu_update_timer.timeOut()) {
            bno_imu.operate();
            imu_update_timer.reset();

            body_orientation = bno_imu.getOrientation();
        }

        // Update the legs
        bool leg_success = true;
        if (leg_update_timer.timeOut()) {
            // Updates each leg
            for (int i = 0; i < NUM_LEGS; i++) {
                if(!(leg[i]->operate())) {
                    leg_success = false;
                    //doError(1);
                    break;
                }
            }
            // Sends signals
            if (leg_success) { sendAllSignals(); }
            leg_update_timer.reset();
        }
    }

    // ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ======== Walking =====================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
    struct TrottingInfo {
        // Defaults
        Rot *body_orientation;
        Point default_leg_position[4];
        Point default_height;
        Point raise_height;
        Point push_height;

        // Movement Request
        float *dx; // These are pointers. They are updated externally and read via getRequested___()
        float *dy;
        float *dyaw;
        Frame frame; // Frame that the movement request was provided in

        // State
        int state;
        Timer timer;
        bool begin_state;
        bool leg_is_swinging[4];

        TrottingInfo() {}

        TrottingInfo(float *x, float *y, float *yaw, Rot *body_orientation_ref) {
            default_leg_position[0] = Point( LENGTH2,  WIDTH2, 0); // All these need to be variablized
            default_leg_position[1] = Point(-LENGTH2,  WIDTH2, 0);
            default_leg_position[2] = Point(-LENGTH2, -WIDTH2, 0);
            default_leg_position[3] = Point( LENGTH2, -WIDTH2, 0);

            leg_is_swinging[0] = false;
            leg_is_swinging[1] = true;
            leg_is_swinging[2] = false;
            leg_is_swinging[3] = true;

            dy = y;
            dx = x;
            dyaw = yaw;
            body_orientation = body_orientation_ref;

            default_height = Point(0, 0, 110);
            raise_height = Point(0, 0, 35);
            push_height = Point(0, 0, 15);

            frame = Frame::BODY;
            state = 0;
            begin_state = true;
        }

        Rot getBodyYaw() {
            return Rot(0, 0, body_orientation->z);
        }

        // Return Desired position in ground frame
        Point getRequestedLegPosition(int i) {
            Rot r = getRequestedRot() + getBodyYaw();
            Point dp = getRequestedTrans();
            if (frame == Frame::BODY) {
                dp /= r;
            }
            return default_leg_position[i] / r + dp;
        }

        // Return basic position. in current heading
        Point getDefaultLegPosition(int i) {
            return default_leg_position[i] / getBodyYaw();
        }

        // Amount to rotate by
        Rot getRequestedRot() {
            return Rot(0, 0, *dyaw);
        }

        Point getRequestedTrans() {
            return Point(*dx, *dy, 0);
        }

        bool legIsSwinging(int i) {
            return leg_is_swinging[i];
        }

        void switchPhase() {
            for (int i = 0; i < NUM_LEGS; i++) {
                leg_is_swinging[i] = !leg_is_swinging[i];
            }
        }
    };

    // Trotting
    TrottingInfo trot_info;

    // Register Request pointers
    void beginTrot(float *x, float *y, float *yaw) {
        trot_info = TrottingInfo(x, y, yaw, &body_orientation);
        trot_info.timer.reset();
    }

    // Register frame in which requests are made
    void setTrotFrame(Frame frame) {
        trot_info.frame = frame;
    }

    // Algorithm 2 - 2 states
    void trot() {
        Rot desired_orientation = body_orientation*0.5;
        desired_orientation.z = body_orientation.z;
        //Serial.print("desired orient: "); desired_orientation.print();

        switch (trot_info.state) {
            case 0: {// Kick legs out, push body forward; maintain balance
                if (trot_info.begin_state) {

                    // Serial.println("State 1: Kick Leg");
                    int period = 400;

                    // Kick 2 feet out
                    for (int i = 0; i < NUM_LEGS; i++) {
                        if (trot_info.legIsSwinging(i)) {
                            leg[i]->setState(FootState::FIXED);
                            leg[i]->setToPositionFromCentroid(trot_info.getRequestedLegPosition(i) + trot_info.raise_height, 
                                                              Frame::GROUND, period*0.99);
                        } else {
                            leg[i]->setState(FootState::PLANTED);
                        }
                    }
                    // Push Body Forward and up
                    setFromCentroid(desired_orientation + trot_info.getRequestedRot()/2, 
                                    trot_info.default_height + trot_info.getRequestedTrans(),// + trot_info.push_height, 
                                    Frame::GROUND, period * 0.99);
                    
                    trot_info.timer.reset(period);
                    trot_info.begin_state = false;
                    break;
                }
               // maintain balance

                // Finished. Switch state.
                if (trot_info.timer.timeOut()) {
                    trot_info.state = 1;
                    trot_info.begin_state = true;
                } else {
                    break;
                }
            }
            case 1: {// Setting feet down
                if (trot_info.begin_state) {
                    // Serial.println("State 2: Return Leg");
                    int period = 400;

                    // Place feet on floor
                    for (int i = 0; i < NUM_LEGS; i++) {
                        if (trot_info.legIsSwinging(i)) {

                            leg[i]->setState(FootState::FIXED);
                            leg[i]->setToPositionFromCentroid(trot_info.getRequestedLegPosition(i), 
                                                              Frame::GROUND, period * 0.99);
                            // leg[i]->setState(FootState::FLOATING);
                            // leg[i]->setToPositionFromBody(trot_info.getRequestedLegPosition(i) - trot_info.default_height, 
                            //                               Frame::GROUND, period*0.99);
                        }
                    }
                    // Continue Pushing Body
                    setFromCentroid(desired_orientation + trot_info.getRequestedRot(), 
                                    trot_info.default_height + trot_info.getRequestedTrans(), 
                                    Frame::GROUND, period * 0.99); // move dog body up a bit
                    
                    trot_info.timer.reset(period);
                    trot_info.begin_state = false;
                    break;
                }

                // Check if legs have hit the ground. Also maintain balance.
                bool legs_hit_ground = false;
                for (int i = 0; i < NUM_LEGS; i++) {
                    if (trot_info.legIsSwinging(i)) {
                        if ((leg[i]->getCurrentFootPositionFromCentroid(Frame::GROUND)).z < 1.5)
                            legs_hit_ground = true;
                    }
                }


                // Legs have completed trajectory. Switch to other legs
                if (trot_info.timer.timeOut() || legs_hit_ground) {
                    trot_info.state = 0;
                    trot_info.begin_state = true;
                    trot_info.switchPhase();
                } else {
                    break;
                }
            }
        }

        operate();Serial.println("");
    }

    bool fixedLegPresent() {
        for (int i = 0; i < NUM_LEGS; i++) {
            if (leg[i]->isFixed()) {
                return true;
            }
        }
        return false;
    }


    void operateAllLegs() {
        for (int i = 0; i < NUM_LEGS; i++) {
            leg[i]->operate();
        }
    }

    bool isIdle() {
        for (int i = 0; i < NUM_LEGS; i++) {
            if (!(leg[i]->isIdle()))
                return false;
        }
        return true;
    }



// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ======== DEBUGGINH =====================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~    
    // Forces a new orientation to be set...mostly for debugging
    void setOrientation(Rot new_orientation) {
        body_orientation = new_orientation;
    }

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

    void printIMU() {
        Serial.print("IMU: "); (bno_imu.getOrientation()).print();
    }

    void printLegs() {
        for (int i = 0; i < NUM_LEGS; i++) {
            Point foot_pos_G = leg[i]->getCurrentFootPositionFromShoulder(Frame::GROUND);
            Serial.print("Leg "); Serial.print(i); Serial.print(": "); foot_pos_G.print();
        }
    }

    void printLegsFromBody() {
        for (int i = 0; i < NUM_LEGS; i++) {
            Point foot_pos_G = leg[i]->getCurrentFootPositionFromBody(Frame::GROUND);
            Serial.print("Leg "); Serial.print(i); Serial.print(": "); foot_pos_G.print();
        }
    }

    void printLeg(int i) {
        Point foot_pos_G = leg[i]->getCurrentFootPositionFromShoulder(Frame::GROUND);
        Serial.print("Leg "); Serial.print(i); Serial.print(": "); foot_pos_G.print();
    }

    void printCentroid() {
        Serial.print("Centoid (oBfG): "); centroid_oBfG.print();
    }

    DogLeg *getLeg(int i) {
        return leg[i];
    }

    RServoDriver *getServoDriver() {
        return &servo_driver;
    }
};


#endif

// Non-Trajectory Walking
    // Body moves by gait length.
    // Leg moves by 2x gait length
    // FUTURE: must account for if leg does not start on ground..."current height"?
//     void step(float direction_yaw, float gait_length) {
//         if (gait_state == 1) {
//             leg[0]->setPLANTED(true);
//             leg[1]->setPLANTED(false);
//             leg[2]->setPLANTED(false);
//             leg[3]->setPLANTED(true);
//         } else {
//             leg[0]->setPLANTED(false);
//             leg[1]->setPLANTED(true);
//             leg[2]->setPLANTED(true);
//             leg[3]->setPLANTED(false);
//         }
//         // Raise moving legs
//         for (int i = 0; i < NUM_LEGS; i++) {
//             if (!(leg[i]->isPLANTED())) {
//               leg[i]->setToPositionFromShoulder(Point(gait_length * cos(direction_yaw DEG), 
//                                                       gait_length * sin(direction_yaw DEG), 
//                                                       -height + STEP_HEIGHT), 
//                                                 Frame::GROUND);
//               leg[i]->sendSignals();
//             }
//         }
//         delay(50);

//         // Tilt body forward/Push PLANTED legs
//         moveByPosition(gait_length * cos(direction_yaw DEG), gait_length * sin(direction_yaw DEG), 0);

//         delay(50);
//         // Put down moving legs
//         for (int i = 0; i < NUM_LEGS; i++) {
//             if (!(leg[i]->isPLANTED())) {
//                 leg[i]->gotoPositionFromShoulder(Point(2*gait_length * cos(direction_yaw DEG), 
//                                                        2*gait_length * sin(direction_yaw DEG), 
//                                                        -height), 
//                                                  Frame::GROUND);
//                 leg[i]->sendSignals();
//             }

//         }
//         gait_state *= -1;
//     }

//     void wave_step(float direction_yaw, float gait_length) {
//     // Serial.println("wave");
//         for (int i = 0; i < NUM_LEGS; i++) {
//             if (i == wave_state) {
//                 leg[i]->setPLANTED(false);
//             } else {
//                 leg[i]->setPLANTED(true);
//             }
//         }
// //         Serial.print("Stepping ");Serial.println(wave_state);
// // printLegs();
//  // Serial.println("Raising Leg");
//         // Raise moving legs
//         for (int i = 0; i < NUM_LEGS; i++) {
//              Serial.println(i);
//             if (!(leg[i]->isPLANTED())) {
//               // Serial.println("ey");
//               // leg[i]->gotoPositionFromShoulder(Point(gait_length * cos(direction_yaw DEG), gait_length * sin(direction_yaw DEG), -height + STEP_HEIGHT));
//               leg[i]->moveByPosition(Point(1.5*gait_length * cos(direction_yaw DEG), 1.5*gait_length * sin(direction_yaw DEG), STEP_HEIGHT), orientation);
//               leg[i]->sendSignals();
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
//             if (!(leg[i]->isPLANTED())) {
//                 leg[i]->moveByPosition(Point(1.5*gait_length * cos(direction_yaw DEG), 1.5*gait_length * sin(direction_yaw DEG), -STEP_HEIGHT), orientation);
//               // leg[i]->gotoPositonFromBody(Point(2*gait_length * cos(direction_yaw DEG), 2*gait_length * sin(direction_yaw DEG), -STEP_HEIGHT), orientation);
//                 leg[i]->sendSignals();
//             }
//         }
//         wave_state = (wave_state + 1)%NUM_LEGS;
//     }