#ifndef __DOG__
#define __DOG__


#include "RServoDriver.h"
#include "Leg.h"

#include <arduino.h>
#include "math2.h"
#include "Point.h"
#include "Rot.h"

// Unless otherwise noted, all "positions" are relative to body origin

// That weird bug where the array values got corrupted...No idea why?? Related to serial prints?

class RobotDog {
    #define LEG_UR_C_ANG_OFS  (4)
    #define LEG_UR_S_ANG_OFS  (-2)
    #define LEG_UR_E_ANG_OFS  (9)
    #define LEG_BR_C_ANG_OFS  (-2.5)
    #define LEG_BR_S_ANG_OFS  (3)
    #define LEG_BR_E_ANG_OFS  (3.5)
    #define LEG_BL_C_ANG_OFS  (-8)
    #define LEG_BL_S_ANG_OFS  (1.5)
    #define LEG_BL_E_ANG_OFS  (0.5)
    #define LEG_UL_C_ANG_OFS  (5)
    #define LEG_UL_S_ANG_OFS  (-4)
    #define LEG_UL_E_ANG_OFS  (-3.5)

    #define DEFAULT_LEG_HEIGHT 100 //mm
    #define WIDTH2 60 // half total width
    #define LENGTH2 112.5 // half total length
    #define STEP_HEIGHT 30

    #define NUM_LEGS 4

    // RServoDriver *servo_driver;
    RServoDriver servo_driver;

    DogLeg leg_ur;
    DogLeg leg_ul;
    DogLeg leg_br;
    DogLeg leg_bl;

// Some array corruption is going on wtf
    DogLeg *leg[4]; //= {&leg_ur, &leg_br, &leg_bl, &leg_ul};

    Rot orientation;
    double height;// useful for getting foot movement height

    double speed;
    int gait_state; // 1 for 13 moving, 2 for 24 moving
    int wave_state;

public:
      // RobotDog() {}

      RobotDog(double max_speed=30) {
        speed = max_speed;
        gait_state = 1;
        wave_state = 0;
       // servo_driver.defaultStartup();
        // servo_driver = ndriver;

        leg[0] = &leg_ur;
        leg[1] = &leg_br;
        leg[2] = &leg_bl;
        leg[3] = &leg_ul;

        height = DEFAULT_LEG_HEIGHT;

        leg_ur = DogLeg(&servo_driver,  0,  1,  2, false, false, Point( LENGTH2,  WIDTH2, 0), DEFAULT_LEG_HEIGHT);
        leg_ur.calibrateServos(LEG_UR_C_ANG_OFS, LEG_UR_S_ANG_OFS, LEG_UR_E_ANG_OFS);

        leg_br = DogLeg(&servo_driver,  4,  5,  6, false,  true, Point(-LENGTH2,  WIDTH2, 0), DEFAULT_LEG_HEIGHT);
        leg_br.calibrateServos(LEG_BR_C_ANG_OFS, LEG_BR_S_ANG_OFS, LEG_BR_E_ANG_OFS);

        leg_bl = DogLeg(&servo_driver,  8,  9, 10,  true,  true, Point(-LENGTH2, -WIDTH2, 0), DEFAULT_LEG_HEIGHT);
        leg_bl.calibrateServos(LEG_BL_C_ANG_OFS, LEG_BL_S_ANG_OFS, LEG_BL_E_ANG_OFS);

        leg_ul = DogLeg(&servo_driver, 12, 13, 14,  true, false, Point( LENGTH2, -WIDTH2, 0), DEFAULT_LEG_HEIGHT);
        leg_ul.calibrateServos(LEG_UL_C_ANG_OFS, LEG_UL_S_ANG_OFS, LEG_UL_E_ANG_OFS);
      }

      void begin() {
        servo_driver.defaultStartup();
        // leg[0]->gotoPositionFromBody(Point(0,0,-DEFAULT_LEG_HEIGHT/2), ROT_ZERO);
        // leg[2]->gotoPositionFromBody(Point(0,0,-DEFAULT_LEG_HEIGHT/2), ROT_ZERO);
      }

      void gotoOrientation(double roll, double pitch, double yaw) {
        Rot new_orientation = Rot(roll, pitch, yaw);
        // Calculate leg positions and check if they're valid
        for (int i = 0; i < NUM_LEGS; i++) {
          if (leg[i]->isAnchored())
            if (!leg[i]->gotoPositionFromBody(leg[i]->getCurrentFootPosition_G(), new_orientation)) {
              doError(i);
              return;
            }
        }
        // If so, actuates and saves values
        sendAnchoredSignals();
        orientation = new_orientation;
      }

      // Moves origin by certain amount in ground frame
      void moveByPosition(double dx, double dy, double dz) {
        // Calculates new positions
        // printLegs();
        for (int i = 0; i < NUM_LEGS; i++) {
          if (leg[i]->isAnchored())
            if (!(leg[i]->moveByPosition(Point(-dx, -dy, -dz), orientation))) {
              doError(i);
              return;
            }
        }
        // If so, actuates and saves values
        sendAnchoredSignals();
        // printLegs();

        height += dz;
      }

      void moveToPose(double roll, double pitch, double yaw, double dx, double dy, double dz) {
          Rot new_orientation = Rot(roll, pitch, yaw);
          // Calculates new positions
          for (int i = 0; i < NUM_LEGS; i++) {
            if (leg[i]->isAnchored())
              if (!(leg[i]->moveByPosition(Point(-dx, -dy, -dz), new_orientation))) {
                doError(i);
                return;
              }
          }
          // If so, actuates and saves values
          sendAnchoredSignals();
          height += dz;
      }


// Non-Trajectory Walking
      // Body moves by gait length.
      // Leg moves by 2x gait length
      // FUTURE: must account for if leg does not start on ground..."current height"?
      void step(double direction_yaw, double gait_length) {
          if (gait_state == 1) {
            leg[0]->setAnchored(true);
            leg[1]->setAnchored(false);
            leg[2]->setAnchored(false);
            leg[3]->setAnchored(true);
          } else {
            leg[0]->setAnchored(false);
            leg[1]->setAnchored(true);
            leg[2]->setAnchored(true);
            leg[3]->setAnchored(false);
          }
          // Raise moving legs
          for (int i = 0; i < NUM_LEGS; i++) {
            if (!(leg[i]->isAnchored())) {
              leg[i]->gotoPositionFromShoulder(Point(gait_length * cos(direction_yaw DEG), gait_length * sin(direction_yaw DEG), -height + STEP_HEIGHT));
              leg[i]->sendSignals();
            }
          }
          delay(50);

          // Tilt body forward/Push anchored legs
          moveByPosition(gait_length * cos(direction_yaw DEG), gait_length * sin(direction_yaw DEG), 0);

           delay(50);
          // Put down moving legs
          for (int i = 0; i < NUM_LEGS; i++) {
            if (!(leg[i]->isAnchored())) {
                leg[i]->gotoPositionFromShoulder(Point(2*gait_length * cos(direction_yaw DEG), 2*gait_length * sin(direction_yaw DEG), -height));
                leg[i]->sendSignals();
            }

          }
          gait_state *= -1;
      }

  void wave_step(double direction_yaw, double gait_length) {
    // Serial.println("wave");
        for (int i = 0; i < NUM_LEGS; i++) {
          if (i == wave_state) {
            leg[i]->setAnchored(false);
          } else {
            leg[i]->setAnchored(true);
          }
        }
//         Serial.print("Stepping ");Serial.println(wave_state);
// printLegs();
 // Serial.println("Raising Leg");
          // Raise moving legs
          for (int i = 0; i < NUM_LEGS; i++) {
            // Serial.println(i);
            if (!(leg[i]->isAnchored())) {
              // Serial.println("ey");
              // leg[i]->gotoPositionFromShoulder(Point(gait_length * cos(direction_yaw DEG), gait_length * sin(direction_yaw DEG), -height + STEP_HEIGHT));
              leg[i]->moveByPosition(Point(1.5*gait_length * cos(direction_yaw DEG), 1.5*gait_length * sin(direction_yaw DEG), STEP_HEIGHT), orientation);
              leg[i]->sendSignals();
            }
            // printLegs();
          }
          // Serial.println("Leg Raised");

          delay(25);
// printLegs();
          // Tilt body forward/Push anchored legs
          moveByPosition(gait_length * cos(direction_yaw DEG), gait_length * sin(direction_yaw DEG), 0);

           // delay(25);
          // Put down moving legs
          for (int i = 0; i < NUM_LEGS; i++) {
            if (!(leg[i]->isAnchored())) {
                // leg[i]->gotoPositionFromShoulder(Point(2*gait_length * cos(direction_yaw DEG), 2*gait_length * sin(direction_yaw DEG), -height));
              leg[i]->moveByPosition(Point(1.5*gait_length * cos(direction_yaw DEG), 1.5*gait_length * sin(direction_yaw DEG), -STEP_HEIGHT), orientation);
              // leg[i]->gotoPositonFromBody(Point(2*gait_length * cos(direction_yaw DEG), 2*gait_length * sin(direction_yaw DEG), -STEP_HEIGHT), orientation);
                leg[i]->sendSignals();
            }
          }
          
// Serial.println("#################");
          wave_state = (wave_state + 1)%NUM_LEGS;
      }

      // Move to original positions and anchors
      void gotoDefaultStance() {
        for (int i = 0; i < NUM_LEGS; i++) {
          leg[i]->gotoPositionFromShoulder(Point(0, 0, -DEFAULT_LEG_HEIGHT));
          leg[i]->setAnchored(true);
        }
        sendAllSignals();
      }

      void gotoDefault() {

      }

      void doError(int e) {
        Serial.print("Dog: Error Occurred: "), Serial.println(e);
      }

      void sendAllSignals() {
        for (int i = 0; i < NUM_LEGS; i++) {
          leg[i]->sendSignals();
        }
      }

      void sendAnchoredSignals() {
        for (int i = 0; i < NUM_LEGS; i++) {
          if (leg[i]->isAnchored()) {
            leg[i]->sendSignals();
          }

        }
      }

      void printLegs() {
        for (int i = 0; i < NUM_LEGS; i++) {
          Point foot_pos_G = leg[i]->getCurrentFootPosition_G();
          Serial.print("Current "),Serial.print(i),Serial.print(": ");
          Serial.print(foot_pos_G.x);
          Serial.print(" "),Serial.print(foot_pos_G.y);
          Serial.print(" "),Serial.println(foot_pos_G.z);
        }
        // Serial.println("Actual______");
        // Point foot_pos_G = leg_ur.getCurrentFootPosition_G();
        //   Serial.print("Current "),Serial.print(0),Serial.print(": "),Serial.print(foot_pos_G.x);
        //   Serial.print(" "),Serial.print(foot_pos_G.y);
        //   Serial.print(" "),Serial.println(foot_pos_G.z);
        //    foot_pos_G = leg_br.getCurrentFootPosition_G();
        //   Serial.print("Current "),Serial.print(1),Serial.print(": "),Serial.print(foot_pos_G.x);
        //   Serial.print(" "),Serial.print(foot_pos_G.y);
        //   Serial.print(" "),Serial.println(foot_pos_G.z);
        //    foot_pos_G = leg_bl.getCurrentFootPosition_G();
        //   Serial.print("Current "),Serial.print(2),Serial.print(": "),Serial.print(foot_pos_G.x);
        //   Serial.print(" "),Serial.print(foot_pos_G.y);
        //   Serial.print(" "),Serial.println(foot_pos_G.z);
        //    foot_pos_G = leg_ul.getCurrentFootPosition_G();
        //   Serial.print("Current "),Serial.print(3),Serial.print(": "),Serial.print(foot_pos_G.x);
        //   Serial.print(" "),Serial.print(foot_pos_G.y);
        //   Serial.print(" "),Serial.println(foot_pos_G.z);
        // Serial.println("$$$$$$");
      }


      void operate() {
      }

};

#endif