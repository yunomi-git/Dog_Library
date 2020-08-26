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

// Unless otherwise noted, all "positions" are relative to body origin...update this

class RobotDog {
    // #define LEG_UR_C_ANG_OFS  (4)
    // #define LEG_UR_S_ANG_OFS  (-2)
    // #define LEG_UR_E_ANG_OFS  (9)
    // #define LEG_BR_C_ANG_OFS  (-2.5)
    // #define LEG_BR_S_ANG_OFS  (3)
    // #define LEG_BR_E_ANG_OFS  (3.5)
    // #define LEG_BL_C_ANG_OFS  (-8)
    // #define LEG_BL_S_ANG_OFS  (1.5)
    // #define LEG_BL_E_ANG_OFS  (0.5)
    // #define LEG_UL_C_ANG_OFS  (5)
    // #define LEG_UL_S_ANG_OFS  (-4)
    // #define LEG_UL_E_ANG_OFS  (-3.5)
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

    #define DEFAULT_LEG_HEIGHT 100 //mm
    #define WIDTH2 60 // half total width
    #define LENGTH2 112.5 // half total length
    #define STEP_HEIGHT 30

    #define IMU_UPDATE_PERIOD 1 // ms
    #define LEG_UPDATE_PERIOD 3 // ms

    #define NUM_LEGS 4

    // Legs
    RServoDriver servo_driver;

    DogLeg leg_ur;
    DogLeg leg_ul;
    DogLeg leg_br;
    DogLeg leg_bl;

    DogLeg *leg[NUM_LEGS]; 

    Timer leg_update_timer;     // in s

    Point COM; // COM relative to default body origin

    // IMU
    IMU bno_imu;
    Timer imu_update_timer;
    bool use_imu;
    Rot desired_orientation;    // Desired orientation of the body

    // Pose/State
    Point centroid_oBfG;        // Centroid location in GROUND frame relative to BODY origin
    Rot body_orientation;       // Actual orientation of the body, relative to stagnant ground frame

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

public:
    RobotDog() {
        // Set up legs
        leg[0] = &leg_ur;
        leg[1] = &leg_br;
        leg[2] = &leg_bl;
        leg[3] = &leg_ul;

        // COM = Point(3, 0, 9.5);
        COM = Point(0,0,0);

        leg_ur = DogLeg(&servo_driver,  0,  1,  2, false, false, &body_orientation, Point( LENGTH2,  WIDTH2, 0) - COM, DEFAULT_LEG_HEIGHT);
        leg_ur.setSignalTables(table_chest_ur, table_shoulder_ur, table_elbow_ur);
        leg_ur.calibrateServos(LEG_UR_C_ANG_OFS, LEG_UR_S_ANG_OFS, LEG_UR_E_ANG_OFS);
        leg_ur.setCentroidRef(&centroid_oBfG);

        leg_br = DogLeg(&servo_driver,  4,  5,  6, false,  true, &body_orientation, Point(-LENGTH2,  WIDTH2, 0) - COM, DEFAULT_LEG_HEIGHT);
        leg_br.setSignalTables(table_chest_br, table_shoulder_br, table_elbow_br);
        leg_br.calibrateServos(LEG_BR_C_ANG_OFS, LEG_BR_S_ANG_OFS, LEG_BR_E_ANG_OFS);
        leg_br.setCentroidRef(&centroid_oBfG);

        leg_bl = DogLeg(&servo_driver,  8,  9, 10,  true,  true, &body_orientation, Point(-LENGTH2, -WIDTH2, 0) - COM, DEFAULT_LEG_HEIGHT);
        leg_bl.setSignalTables(table_chest_bl, table_shoulder_bl, table_elbow_bl);
        leg_bl.calibrateServos(LEG_BL_C_ANG_OFS, LEG_BL_S_ANG_OFS, LEG_BL_E_ANG_OFS);
        leg_bl.setCentroidRef(&centroid_oBfG);

        leg_ul = DogLeg(&servo_driver, 12, 13, 14,  true, false, &body_orientation, Point( LENGTH2, -WIDTH2, 0) - COM, DEFAULT_LEG_HEIGHT);
        leg_ul.setSignalTables(table_chest_ul, table_shoulder_ul, table_elbow_ul);
        leg_ul.calibrateServos(LEG_UL_C_ANG_OFS, LEG_UL_S_ANG_OFS, LEG_UL_E_ANG_OFS);
        leg_ul.setCentroidRef(&centroid_oBfG);


        // Other setup
        desired_orientation = ROT_ZERO;
        use_imu = true;

        updateCentroid();
    }

    void begin() {
        servo_driver.defaultStartup();

        if (use_imu) {
            bno_imu.setCollectionMode(IMU::CONTINUOUS);
            bno_imu.defaultStartup();
        }

        leg_update_timer.reset(LEG_UPDATE_PERIOD);
        imu_update_timer.reset(IMU_UPDATE_PERIOD);
    }

    void tareIMU() {
        bno_imu.tareOrientation();
    }

    void useIMU(bool to_use) {
        use_imu = to_use;
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
    // Only ANCHORED/FIXED legs get moved.
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
            if (leg[i]->isAnchored()) {
                new_leg_position_oBfG = leg[i]->getCurrentFootPositionFromBody(Frame::GROUND) * r; // Rotate body to the desired orientation
                new_leg_position_oBfG -= dp; // Translate the body accordingly
                leg[i]->setToPositionFromBody(new_leg_position_oBfG, Frame::GROUND, time);
            }
        }
    }

    // Coordinates based on centroid of anchored feet
    // Centroid is the centroid of the polygon created by the ANCHORED feet, at the default height
    // Kinematically moves dog body to a new orientation/position relative to current pose.
    // New orientation may not be exactly equivalent to measured orientation.
    // Only ANCHORED/FIXED legs get moved.
    // Implementation: 
    // @param r:                  Desired body orientation
    // @param p_oC:               Desired body position, relative to centroid
    // @param frame:              Reference frame in which dp is given. If BODY is chosen, movement based on *DESIRED* body orientation.
    // @param time:               Total movement time desired to move the foot from its current position to the new position.
    //                            Set to 0/TIME_INSTANT for max speed.
    void setFromCentroid(Rot r, Point p_oCf, Frame frame, float time=TIME_INSTANT) {
        // Calculate distance to move, in GROUND frame
        Point dp_fB;
        updateCentroid();

        if (frame == Frame::GROUND) {
            dp_fB = (p_oCf + centroid_oBfG) * r;
        } else { // BODY frame
            dp_fB = p_oCf + centroid_oBfG * r;
        }

        desired_orientation = r;

        // Calculates new positions
        Point new_leg_position_oBfB;
        for (int i = 0; i < NUM_LEGS; i++) {
            if (leg[i]->isAnchored()) {
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

    // Centroid relative to BODY origin, in GROUND frame, based on foot positions
    // z height is default leg heigh
    void updateCentroid() {
        int num_anchored = 0;
        float lowest_leg_height = 0;
        Point centroid = POINT_ZERO;
        for (int i = 0; i < NUM_LEGS; i++) {
            if (leg[i]->isAnchored()) {
                Point leg_pos_oBfG = leg[i]->getCurrentFootPositionFromBody(Frame::GROUND);
                centroid += leg_pos_oBfG; // get x, y of centroid
                if (leg_pos_oBfG.z < lowest_leg_height)
                    lowest_leg_height = leg_pos_oBfG.z;
                num_anchored++;                
            }
        }
        
        if (num_anchored == 0) {
            doError(2);
            return;
        } 

        centroid /= num_anchored;
        centroid.z = lowest_leg_height; // Calculate actual centroid
        
        centroid_oBfG = centroid;
    }

    Point getCentroid() {
        updateCentroid();
        return centroid_oBfG;
    }

    // Move to original positions and anchors
    void gotoDefaultStance() {
        for (int i = 0; i < NUM_LEGS; i++) {
            leg[i]->setToPositionFromShoulder(Point(0, 0, -DEFAULT_LEG_HEIGHT));
            leg[i]->setState(CoordinationState::ANCHORED);
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
        if (use_imu) {
            // Update the IMU and centroid
            if (imu_update_timer.timeOut()) {
                bno_imu.operate();
                imu_update_timer.reset();

                body_orientation = bno_imu.getOrientation();

                // if (fixedLegPresent()) {
                    updateCentroid();
                // }
            }
        } else { // No IMU has not been fully tested, especially when walking 
            if (imu_update_timer.timeOut()) {
                if (isIdle()) {
                    body_orientation = desired_orientation;
                }
                imu_update_timer.reset();
            }
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
                            leg[i]->setState(CoordinationState::FIXED);
                            leg[i]->setToPositionFromCentroid(trot_info.getRequestedLegPosition(i) + trot_info.raise_height, 
                                                              Frame::GROUND, period*0.99);
                        } else {
                            leg[i]->setState(CoordinationState::ANCHORED);
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

                            leg[i]->setState(CoordinationState::FIXED);
                            leg[i]->setToPositionFromCentroid(trot_info.getRequestedLegPosition(i), 
                                                              Frame::GROUND, period * 0.99);
                            // leg[i]->setState(CoordinationState::FLOATING);
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



/*
    void trot() {
        Rot desired_orientation = body_orientation*0.9;
        desired_orientation.z = body_orientation.z;
        //Serial.print("desired orient: "); desired_orientation.print();

        switch (trot_info.state) {
            case 0: {// Pull body to center, Raise legs up
                if (trot_info.begin_state) {
                    // Serial.println("State 0: Raise Leg");
                    int period = 70;
                    // Raise 2 feet up
                    for (int i = 0; i < NUM_LEGS; i++) {
                        if (trot_info.legIsSwinging(i)) {
                            leg[i]->setState(CoordinationState::FIXED);
                            leg[i]->setToPositionFromCentroid(trot_info.getDefaultLegPosition(i) + trot_info.raise_height, 
                                                              Frame::GROUND, period*0.99);
                        } else {
                            leg[i]->setState(CoordinationState::ANCHORED);
                        }
                    }
                    // Pull body
                    setFromCentroid(desired_orientation, 
                                    trot_info.default_height, 
                                    Frame::GROUND, period * 0.99); // move dog body up a bit
                    
                    trot_info.timer.reset(period);
                    trot_info.begin_state = false;
                    break;
                }
                // Maintain Balance...not ready yet

                // Finished. Switch state.
                if (trot_info.timer.timeOut()) {
                    trot_info.state = 1;
                    trot_info.begin_state = true;
                } else {
                    break;
                }
            }
            case 1: {// Kick legs out, push body forward; maintain balance
                if (trot_info.begin_state) {
                    // Serial.println("State 1: Kick Leg");
                    int period = 70;

                    // Kick 2 feet out
                    for (int i = 0; i < NUM_LEGS; i++) {
                        if (trot_info.legIsSwinging(i)) {
                            // leg[i]->setToPositionFromCentroid(trot_info.getRequestedLegPosition(i) + trot_info.raise_height, 
                            //                                   Frame::GROUND, period * 0.99);
                            leg[i]->setState(CoordinationState::FLOATING);
                            leg[i]->setToPositionFromBody(trot_info.getRequestedLegPosition(i) - trot_info.default_height + trot_info.raise_height, 
                                                          Frame::GROUND, period*0.99);
                        }
                    }
                    // Push Body Forward and up
                    setFromCentroid(desired_orientation+ trot_info.getRequestedRot(), 
                                    trot_info.default_height + trot_info.push_height, 
                                    Frame::GROUND, period * 0.99);
                    
                    trot_info.timer.reset(period);
                    trot_info.begin_state = false;
                    break;
                }
               // maintain balance

                // Finished. Switch state.
                if (trot_info.timer.timeOut()) {
                    trot_info.state = 2;
                    trot_info.begin_state = true;
                } else {
                    break;
                }
            }
            case 2: {// Setting feet down
                if (trot_info.begin_state) {
                    // Serial.println("State 2: Return Leg");
                    int period = 140;

                    // Place feet on floor
                    for (int i = 0; i < NUM_LEGS; i++) {
                        if (trot_info.legIsSwinging(i)) {

                            leg[i]->setState(CoordinationState::FIXED);
                            leg[i]->setToPositionFromCentroid(trot_info.getRequestedLegPosition(i), 
                                                              Frame::GROUND, period * 0.99);
                            // leg[i]->setState(CoordinationState::FLOATING);
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
            // case 3: {// Setting feet down
            //     if (trot_info.begin_state) {
            //         // Serial.println("State 3: Reanchor");
            //         int period = (40);

            //         // Place feet on floor
            //         for (int i = 0; i < NUM_LEGS; i++) {
            //             leg[i]->setState(CoordinationState::ANCHORED);
            //         }
            //         // Set Body Down
            //         Rot desired_orientation = body_orientation * (0.7);
            //         desired_orientation.z = body_orientation.z;
            //         setFromCentroid(desired_orientation,  
            //                         trot_info.default_height, 
            //                         Frame::GROUND, period * 0.99);
            //         trot_info.timer.reset(period);
            //         trot_info.begin_state = false;
            //         break;
            //     }

            //     // Legs have completed trajectory. Switch to other legs
            //     if (trot_info.timer.timeOut()) {
            //         trot_info.state = 0;
            //         trot_info.begin_state = true;
            //     } else {
            //         break;
            //     }
            // }
        }

        operate();
    }
*/
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
        Serial.print("Dog ERROR: ");
        switch (e) {
            case 1: Serial.println("Leg IKIN invalid."); break;
            case 2: Serial.println("No Anchored Points."); break;
        } 
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

   // Height in the ground frame
    // float getCurrentHeight() {
    //     int num_anchored = 0;
    //     Point centroid = POINT_ZERO;
    //     for (int i = 0; i < NUM_LEGS; i++) {
    //         centroid += leg[i]->getCurrentFootPositionFromBody(Frame::GROUND);
    //         num_anchored++;
    //     }
        
    //     if (num_anchored > 0) {
    //         centroid /= num_anchored;
    //         centroid.z = 0;
    //     }
    // }


// Non-Trajectory Walking
    // Body moves by gait length.
    // Leg moves by 2x gait length
    // FUTURE: must account for if leg does not start on ground..."current height"?
//     void step(float direction_yaw, float gait_length) {
//         if (gait_state == 1) {
//             leg[0]->setAnchored(true);
//             leg[1]->setAnchored(false);
//             leg[2]->setAnchored(false);
//             leg[3]->setAnchored(true);
//         } else {
//             leg[0]->setAnchored(false);
//             leg[1]->setAnchored(true);
//             leg[2]->setAnchored(true);
//             leg[3]->setAnchored(false);
//         }
//         // Raise moving legs
//         for (int i = 0; i < NUM_LEGS; i++) {
//             if (!(leg[i]->isAnchored())) {
//               leg[i]->setToPositionFromShoulder(Point(gait_length * cos(direction_yaw DEG), 
//                                                       gait_length * sin(direction_yaw DEG), 
//                                                       -height + STEP_HEIGHT), 
//                                                 Frame::GROUND);
//               leg[i]->sendSignals();
//             }
//         }
//         delay(50);

//         // Tilt body forward/Push anchored legs
//         moveByPosition(gait_length * cos(direction_yaw DEG), gait_length * sin(direction_yaw DEG), 0);

//         delay(50);
//         // Put down moving legs
//         for (int i = 0; i < NUM_LEGS; i++) {
//             if (!(leg[i]->isAnchored())) {
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
//                 leg[i]->setAnchored(false);
//             } else {
//                 leg[i]->setAnchored(true);
//             }
//         }
// //         Serial.print("Stepping ");Serial.println(wave_state);
// // printLegs();
//  // Serial.println("Raising Leg");
//         // Raise moving legs
//         for (int i = 0; i < NUM_LEGS; i++) {
//              Serial.println(i);
//             if (!(leg[i]->isAnchored())) {
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
//           // Tilt body forward/Push anchored legs
//         moveByPosition(gait_length * cos(direction_yaw DEG), gait_length * sin(direction_yaw DEG), 0);

//         // delay(25);
//         // Put down moving legs
//         for (int i = 0; i < NUM_LEGS; i++) {
//             if (!(leg[i]->isAnchored())) {
//                 leg[i]->moveByPosition(Point(1.5*gait_length * cos(direction_yaw DEG), 1.5*gait_length * sin(direction_yaw DEG), -STEP_HEIGHT), orientation);
//               // leg[i]->gotoPositonFromBody(Point(2*gait_length * cos(direction_yaw DEG), 2*gait_length * sin(direction_yaw DEG), -STEP_HEIGHT), orientation);
//                 leg[i]->sendSignals();
//             }
//         }
//         wave_state = (wave_state + 1)%NUM_LEGS;
//     }



    // void setToOrientation(Rot new_orientation, float time=TIME_INSTANT) {
    //     body_orientation = new_orientation;
    //     // Calculate leg positions and check if they're valid
    //     for (int i = 0; i < NUM_LEGS; i++) {
    //         if (leg[i]->isAnchored())
    //             leg[i]->setByPosition(POINT_ZERO, Frame::GROUND, time);
    //     }
    //     // If so, actuates and saves values
    //     sendAnchoredSignals();
    //     orientation = new_orientation;
    // }

    // // Moves origin by certain amount in ground frame
    // void moveByPosition(Point dp, float time=TIME_INSTANT) {
    //     // Calculates new positions
    //     for (int i = 0; i < NUM_LEGS; i++) {
    //         if (leg[i]->isAnchored())
    //             leg[i]->moveByPosition(dp * -1, Frame::GROUND, time);
                
    //     }
    //     // If so, actuates and saves values
    //     sendAnchoredSignals();
    //     // printLegs();

    //     height += dz;
    // }

    // void sendAnchoredSignals() {
    //     for (int i = 0; i < NUM_LEGS; i++) {
    //         if (leg[i]->isAnchored()) {
    //             leg[i]->sendSignals();
    //         }
    //     }
    // }