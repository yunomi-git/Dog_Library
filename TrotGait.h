#ifndef __TROT
#define __TROT__

#include "Dog.h"

// #define DEBUG

// TODO: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// -
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// USAGE DETAILS
/*
 */

/**
* IMPLEMENTATION DETAILS

*/



enum class Frame {GROUND, BODY};

class RobotDog {
private:
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