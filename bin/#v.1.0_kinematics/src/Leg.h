#ifndef __DOGLEG__
#define __DOGLEG__

#include <math.h>
#include <math2.h>
#include <arduino.h>
#include "RServoDriver.h"
#include <Point.h>
#include <Rot.h>
 // #define DEBUG

// Dog leg
// Calculates inverse kinematics to move foot to a desired position.
// Desired foot position can be:
//  - Relative to the shoulder, in the body frame
//  - Relative to the body origin, in the ground/stationary frame
// 
// Reference Leg:
//  -------
//  |  x  |  front
//  ------*
//   right    
// x: body origin. *: shoulder origin
//
// Reference frame of COM
//   ^ +z (up)
//   |
//   . --> +x (front)
//   +y (right)
//
// Chest: servo about x axis
// Shoulder: servo about y axis; upper arm
// Elbow: servo parallel to y axis; forearm
// 
// Mounting Position: Distance of shoulder from body center

// Trajectory: Allows speed specification.
// Fixed speed. Not polynomial, because already using a servo
// 
// TODO a +z offset to account for foot geometry...can ignore?
// TODO add "offset from mounting point" or "offset from current point" option for motion
// TODO finish the frame choice

enum class Frame {GROUND, BODY};

class DogLeg {
    #define CHEST_OFFSET_MAIN 0
    #define SHOULDER_OFFSET_MAIN -15
    #define ELBOW_OFFSET_MAIN -125

    #define WISHBONE_ANGLE 155

    #define L_FOREARM     75   // mm
    #define L_UPPERARM    75   // mm
    #define FOOT_RADIUS   5     // mm   
    #define L_DEFAULT_HEIGHT 100 // mm
    #define DEFAULT_SPEED 5     // mm/s

	  int elbow_chan; // lower joint
	  int shoulder_chan; // upper joint. reversed to elbow
	  int chest_chan; // "roll"

    RServoDriver *servo_driver;

    bool is_anchored; // "Stuck" to the point in space in kinematic calculations, ie does not move with body

    // ==================================================
    // ============= Parameters =========================
    // ==================================================
	  double default_length;
    Point mounting_pos_B; // Mounting point relative to body origin IN BODY FRAME ... AKA the offset
    Point foot_pos_default_G;

    // ==================================================
    // ============= Coordination =======================
    // ==================================================
    // Holds a triplet of information for the angles of the three servos
    struct servo_angles {
    	double chest;
	    double shoulder;
	    double elbow;

	    servo_angles() {
	    	chest = 0;
	    	shoulder = 0;
	    	elbow = 0;
	    }
	    servo_angles(double c, double s, double e) {
	    	chest = c;
	    	shoulder = s;
	    	elbow = e;
	    }
    };

    servo_angles angles_candidate;
    Point foot_pos_candidate_G;

    // ==================================================
    // ============= Trajectory =========================
    // ==================================================
    double cur_speed;   // Speed in mm/s. If speed is 0, movement is instantaneous
    Point cur_foot_pos_G; // Position relative to the body origin IN GROUND FRAME

    Point trajectory_G; // Path vector from the starting position to the goal position in ground frame. Set to POINT_ZERO if not currently in trajectory
    Point goal_foot_pos_G; // Foot position at the end of the trajectory
    double traj_start_time; // Starting time of the trajectory
    double traj_total_time; // Total time of the trajectory
public:

    DogLeg() {}

    DogLeg(RServoDriver *ndriver, int p1, int p2, int p3, bool flip_lr, bool flip_fb, 
           Point mounting_point=POINT_ZERO, double starting_height=L_DEFAULT_HEIGHT, double speed=DEFAULT_SPEED) {
        is_anchored = true;

        servo_driver = ndriver; 
      
        elbow_chan = p3;
        servo_driver->setOffset(elbow_chan, ELBOW_OFFSET_MAIN);

        shoulder_chan = p2;
        servo_driver->setOffset(shoulder_chan, SHOULDER_OFFSET_MAIN);

        chest_chan = p1;
        servo_driver->setOffset(chest_chan, CHEST_OFFSET_MAIN);

        if (flip_fb) {
          servo_driver->reverseDirection(chest_chan);
        } 
        if (flip_lr) {
          servo_driver->reverseDirection(elbow_chan);
          servo_driver->reverseDirection(shoulder_chan);
        }

        mounting_pos_B = mounting_point;
        foot_pos_default_G = Point(0, 0, -starting_height) + mounting_pos_B;
        foot_pos_candidate_G = foot_pos_default_G;
        gotoPositionFromBody(foot_pos_default_G);


        cur_speed = DEFAULT_SPEED;
        cur_foot_pos_G = foot_pos_default_G;

        trajectory_G = POINT_ZERO;
        goal_foot_pos_G = POINT_ZERO;
        traj_start_time = 0; 
        traj_total_time = 0; 
    }

      // Additional offsets to servo angles...MAKE SURE MEASURED AMOUNT IS CONSISTENT WITH DIRECTION
    void calibrateServos(double off1, double off2, double off3) {
        servo_driver->setOffset(elbow_chan, ELBOW_OFFSET_MAIN + off3);
        servo_driver->setOffset(shoulder_chan, SHOULDER_OFFSET_MAIN + off2);
        servo_driver->setOffset(chest_chan, CHEST_OFFSET_MAIN + off1);
    }

    bool isAnchored() {
    	return is_anchored;
    }

    void setAnchored(bool to_anchor) {
    	is_anchored = to_anchor;
    }

    void gotoAngles(double chest, double shoulder, double elbow) {
    	angles_candidate = servo_angles(chest, shoulder, elbow);
    	sendSignals();
    }

    Point getCurrentFootPosition_G() {
    	return cur_foot_pos_G;
    }
private:
      // Commands servos to replicate this state
      // input degrees for angles
      // Values RELATIVE TO MOUNTING POINT
    bool gotoState(double l_arm_eff, double angle_shoulder_eff, double angle_chest) {
        // Angle from forearm to upper arm. 
        // Note: acos is always positive, so joint never inverts!
        double angle_elbow_relative = acos((L_FOREARM*L_FOREARM + L_UPPERARM*L_UPPERARM - l_arm_eff*l_arm_eff)
                                            /(2 * L_FOREARM * L_UPPERARM)) RAD; 

        // Angle of shoulder
        //double angle_shoulder = angle_shoulder_eff - asin(L_FOREARM / l_arm_eff * sin(angle_elbow_relative DEG)) RAD;
        double angle_shoulder = angle_shoulder_eff - (180.0 - angle_elbow_relative) * 0.5; // optimization since arm lengths are the same

        // Angle of elbow
        double angle_elbow = 180 + angle_shoulder - angle_elbow_relative;
        angle_elbow -= WISHBONE_ANGLE; // To get servo angle

        // Angle of chest as is

#ifdef DEBUG
        Serial.print("Chest: "); Serial.print(angle_chest); 
        Serial.print(" | Shoulder: "); Serial.print(angle_shoulder);
        Serial.print(" | Elbow: "); Serial.println(angle_elbow);
#endif

        if (WITHIN(angle_chest, -75, 75) && 
         	WITHIN(angle_elbow, -185, -67) && 
         	WITHIN(angle_shoulder - angle_elbow, 25, 155)) {
         	angles_candidate = servo_angles(angle_chest, angle_shoulder, angle_elbow);
#ifdef DEBUG
         	Serial.println("Leg: Angle Succeeded");
#endif
         	return true;
        } else {
#ifdef DEBUG
         	Serial.println("Leg: Angle failed");
#endif
         	return false;
        }
    }

	// Moves to the position RELATIVE TO MOUNTING POINT
	bool gotoPosition(double x, double y, double z) {
	    // x_ and z_ are the x and z coordinates in the reference plane coincident with the arm plane
	    double x_ = x;
	    double z_ = sqrt(y*y + z*z);

	    // Converts values to state
	    double l_arm_eff = sqrt(x_*x_ + z_*z_);
	    double angle_shoulder_eff = atan2(x_, z_) RAD;
	    double chest_angle = -atan2(y, -z) RAD;

	#ifdef DEBUG
		Serial.print("Eff Length: "); Serial.print(l_arm_eff); Serial.print(" | Shoulder Ang Eff: "); Serial.println(angle_shoulder_eff);
	#endif
	      
	    // Calls state movement to solve
	    return gotoState(l_arm_eff, angle_shoulder_eff, chest_angle);
	}

    // Moves foot to a point in the BODY frame relative to the shoulder position.
    bool gotoPosition(Point p) {
    	return gotoPosition(p.x, p.y, p.z);
    }

public:
    // USE THESE THREE TO MOVE ON DOG BODY
    // Moves foot to the point in the GROUND FRAME, relative to the body origin
    // @param orientation: Body orientation, if using body frame
    bool gotoPositionFromBody(Point new_foot_pos_from_body_G, Rot orientation=ROT_ZERO, Frame frame=Frame::GROUND, double speed=DEFAULT_SPEED) {
      	traj_start_time = millis()/1000;
        // Calculate desired position
        Point p_foot_to_shoulder_B = new_foot_pos_from_body_G * orientation - mounting_pos_B;
        // Save candidate position
      	foot_pos_candidate_G = new_foot_pos_from_body_G;
        // Do IKIN
      	return gotoPosition(p_foot_to_shoulder_B);
    }

    // Moves the foot by the amount IN THE GROUND FRAME, relative to its original position
    bool moveByPosition(Point d_foot_pos_G, Rot orientation=ROT_ZERO, Frame frame=Frame::GROUND, double speed=DEFAULT_SPEED) {
      	return gotoPositionFromBody(cur_foot_pos_G + d_foot_pos_G, orientation, frame, speed);
    }

    // Moves the foot to the position IN THE BODY FRAME, relative to the shoulder origin
    bool gotoPositionFromShoulder(Point new_foot_pos_from_shoulder_B, Rot orientation=ROT_ZERO, Frame frame=Frame::GROUND, double speed=DEFAULT_SPEED) {
      traj_start_time = millis()/1000;
      // Desired position as is
      // Calculate/save candidate position
    	foot_pos_candidate_G = (new_foot_pos_from_shoulder_B + mounting_pos_B).inv_rotate(orientation);
      // Do IKIN
    	return gotoPosition(new_foot_pos_from_shoulder_B);
    }

    // Sends saved angles to servos
	  void sendSignals() {
	  	cur_foot_pos_G = foot_pos_candidate_G;
	    servo_driver->gotoAngle(chest_chan, angles_candidate.chest);
	    servo_driver->gotoAngle(shoulder_chan, angles_candidate.shoulder);
	    servo_driver->gotoAngle(elbow_chan, angles_candidate.elbow);
	  }

    // Moves the foot along its current trajectory at a fixed speed
    // Does nothing if no current trajectory
    void operate() {
        // if (trajectory_G)
        //   double progress = (millis() - traj_start_time)/traj_total_time;
        //   if (progress > 1) {
        //     progress = 1;
        //     trajectory = done;
        //   }
        //   curr_position = final_position - trajectory * (1 - progress);
    }


};

#endif