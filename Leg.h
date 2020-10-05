#ifndef __DOGLEG__
#define __DOGLEG__

#include <math.h>
#include <math2.h>
#include <arduino.h>
#include "RServoDriver.h"
#include <Point.h>
#include <Rot.h>
#include "Trajectory.h"

// TODO: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// - Kinematics are fucked up because of frame switch...rederive
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Dog leg on a body

// USAGE DETAILS
// Calculates inverse kinematics to move foot to a desired position.
// Desired foot position must be:
//  - Relative to the body origin or shoulder origin
//  - In body frame
//
// COORDINATE FRAME
// Reference Leg (top view):
//  ------*
//  |  o  |  front
//  -------
//   right    
// o: body origin. *: shoulder origin
//
// Reference frame of COM (side view):
//   ^ +z (up)
//   |
//   x --> +x (front)
//   +y (left)
//
// - Mounting Position: Distance to shoulder from body center, BODY frame


/**
* IMPLEMENTATION DETAILS
* - All points are in BODY FRAME and RELATIVE TO SHOULDER ORIGIN unless otherwise noted
* 
* Chest: servo about x axis
* Shoulder: servo about y axis; upper arm
* Elbow: servo parallel to y axis; forearm
* 
* - Trajectory: Allows speed specification.
*   Fixed speed in trajectory. Not polynomial, because already using a servo
*
* - Inverse Kinematics ALWAYS performed in BODY frame, relative to SHOULDER
*/


enum class OriginReference{BODY, SHOULDER};

#define TIME_INSTANT 0

class DogLeg {
    #define CHEST_OFFSET_MAIN 0
    #define SHOULDER_OFFSET_MAIN 15
    #define ELBOW_OFFSET_MAIN 125

    #define WISHBONE_ANGLE 155

    #define L_FOREARM     75   // mm
    #define L_UPPERARM    75   // mm
    #define FOOT_RADIUS   5     // mm   
    #define L_DEFAULT_HEIGHT 100 // mm
    #define L_CHEST_DEFAULT 13.97 // mm

    #define UPDATE_RATE 5 // ms
    #define TRACKING_SPEED 40 // mm/s Speed at which fixed leg attempts to correct itself
    #define MAX_SPEED 140.0 // mm/s

    #define FOOT_POS_DEFAULT Point(0, 0, -90) // Default position if no other position set


    // ==================================================
    // ============= Internal ===========================
    // ==================================================

    // ============== SIGNALS ===========================
    // PWM driver signal channels
	int elbow_chan; // lower joint
	int shoulder_chan; // upper joint. reversed to elbow
	int chest_chan; // "roll"

    RServoDriver *servo_driver;

    // ============= Trajectory =========================
    // Holds a foot position and associated servo angles from IKin
    // Body frame, relative to shoulder
    struct IkinInfo {
        Point foot_pos; // Body frame, from shoulder
    	float chest_angle;
	    float shoulder_angle;
	    float elbow_angle;
	    bool is_valid;

	    IkinInfo() {
	    	is_valid = false;
	    }

	    void reset() {
            foot_pos = POINT_ZERO;
	    	chest_angle = 0;
	    	shoulder_angle = 0;
	    	elbow_angle = 0;
	    	is_valid = false;
	    }
    };

    TrajectoryInfo<Point> trajectory_oS;
    IkinInfo next_foot_kinematics; // IKIN info for a working leg position. transient. shoulder origin, body frame

    // =========== Parameters ====================
    int id;
    float L_CHEST;
    Point default_foot_position_oS; // The position the foot goes to when it "resets" its position. Shoulder origin, Body frame
    							 // Assume the dog starts up, or will soon start up, at this position

    // ==================================================
    // ============= STATE =============================
    // ==================================================
    Point mounting_pos; // Mounting point relative to body origin IN BODY FRAME
    // $$ Manually Set
    Point set_foot_position; // Position relative to the shoulder origin. BODY frame 

public:
    DogLeg() {}

    // default_position: default position relative to body origin
    DogLeg(RServoDriver *ndriver, int p1, int p2, int p3, Point mounting_point, Point n_default_position_oB) {
    	id = -1;
    	// Set parameters
    	L_CHEST = L_CHEST_DEFAULT;

        // Signal info
        servo_driver = ndriver; 
      
        elbow_chan = p3;
        servo_driver->setOffset(elbow_chan, ELBOW_OFFSET_MAIN);

        shoulder_chan = p2;
        servo_driver->setOffset(shoulder_chan, SHOULDER_OFFSET_MAIN);

        chest_chan = p1;
        servo_driver->setOffset(chest_chan, CHEST_OFFSET_MAIN);

        // Parameters
        mounting_pos = mounting_point;
        default_foot_position_oS = n_default_position_oB - mounting_pos;

        // Accessed
        set_foot_position = default_foot_position_oS;
    }

    void setMountingPoint(Point n_mounting_point) {
    	mounting_pos = n_mounting_point;
    }

    void setDefaultPosition(Point n_default_position_oB) {
    	default_foot_position_oS = n_default_position_oB;
    }

    void flipFB() {
    	servo_driver->reverseDirection(chest_chan);
    }

    void flipLR() {
		servo_driver->reverseDirection(elbow_chan);
        servo_driver->reverseDirection(shoulder_chan);
        L_CHEST *= -1;
    }

      // Additional offsets to servo angles
    void calibrateServos(float off1, float off2, float off3) {
        servo_driver->setOffset(elbow_chan, ELBOW_OFFSET_MAIN + off3);
        servo_driver->setOffset(shoulder_chan, SHOULDER_OFFSET_MAIN + off2);
        servo_driver->setOffset(chest_chan, CHEST_OFFSET_MAIN + off1);
    }

    void setSignalTables(int *chest_table, int *shoulder_table, int *elbow_table) {
    	servo_driver->getServo(chest_chan)->setTable(chest_table);
    	servo_driver->getServo(shoulder_chan)->setTable(shoulder_table);
    	servo_driver->getServo(elbow_chan)->setTable(elbow_table);
    }

    void setID(int n_id) {
    	id = n_id;
    }




// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ======== INVERSE KINEMATICS ============~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
private:
	IkinInfo inverseKinematics(float x, float y, float z) {
		IkinInfo kinematics_info;
        // x_ and z_ are the x and z coordinates in the reference plane coincident with the arm plane
        float x_ = x;
        float z_ = -sqrt(y*y + z*z - L_CHEST*L_CHEST);

        // Solve for chest angle
        float angle_chest = atan2(y, -z) RAD - atan2(L_CHEST, -z_) RAD;

  	    // Solve for shoulder, elbow angles      
        // Note: acos is always positive, so joint never inverts!
        float effective_leg_length = sqrt(x_*x_ + z_*z_);
        float effective_leg_angle = -atan2(x_, -z_) RAD;
        float interior_elbow_angle = acos((L_FOREARM*L_FOREARM + L_UPPERARM*L_UPPERARM - effective_leg_length*effective_leg_length) /(2 * L_FOREARM * L_UPPERARM)) RAD; 
        float angle_shoulder = asin(L_FOREARM / effective_leg_length * sin(interior_elbow_angle DEG)) RAD + effective_leg_angle;
        float angle_elbow = -(180 - angle_shoulder - interior_elbow_angle) + WISHBONE_ANGLE; // servo angle is offset from actual elbow by WISHBONE

        // Check if returned angles are feasible
        bool ikin_err = false;
        if (!WITHIN(angle_chest, -75, 75)) 					{Serial.print("IKIN ERR: Chest at "); 			 Serial.println(angle_chest); 					ikin_err = true;}
        if (!WITHIN(angle_elbow,  67, 185)) 				{Serial.print("IKIN ERR: Elbow at "); 			 Serial.println(angle_elbow); 					ikin_err = true;}
        if (!WITHIN(angle_elbow - angle_shoulder, 20, 160)) {Serial.print("IKIN ERR: Shoulder - Elbow at "); Serial.println(angle_shoulder - angle_elbow);  ikin_err = true;}
        
        // Saves the calculated values (as appropriate)
        if (ikin_err) {
        	if (id != -1) {
        		Serial.print("Leg "); Serial.print(id); Serial.print(": ");
        	}
        	Serial.print("Requested Point: "); Point(x, y, z).print();
        	kinematics_info.is_valid = false;
        } else {
        	kinematics_info.foot_pos = Point(x, y, z);
            kinematics_info.chest_angle = angle_chest;
            kinematics_info.shoulder_angle = angle_shoulder;
            kinematics_info.elbow_angle = angle_elbow;
            kinematics_info.is_valid = true;
        }

        return kinematics_info;
    }

    // Moves foot to a point in the BODY frame relative to the SHOULDER position.
    IkinInfo inverseKinematics(Point p) {
    	return inverseKinematics(p.x, p.y, p.z);
    }



public:

// ========================================
// ====== STATE ACCESSORS =======
// ========================================
    Point getPosition_oBfB() {
    	if (!set_foot_position) return POINT_NULL;
        return set_foot_position + mounting_pos;
    }

    Point getMountingPoint() {
    	return mounting_pos;
    }

    Point getDefaultPosition_oBfB() {
    	return default_foot_position_oS + mounting_pos;
    }

    int getID() {
    	return id;
    }
// ========================================
// ====== MOTION COMMANDS/ACCESSORS =======
// ========================================
// Commands
    // Moves the foot to the position in the requested frame, relative to the SHOULDER origin
    // Implementation: Set up trajectory information so that operate() can move the foot
    // @param new_foot_pos:       Foot position (in mm) relative to SHOULDER origin, in BODY frame
    // @param time:               Total movement time desired to move the foot from its current position to the new position.
    //                            Set to 0/TIME_INSTANT for max speed.
    void moveToPositionFromShoulder(Point new_foot_pos, float time=TIME_INSTANT) {       
        // Set up trajectory info based on leg state
        trajectory_oS.begin(new_foot_pos, time);
    }

    // Moves the foot to the position in the requested frame, relative to the BODY origin
    // Implementation: Sets up information to pass to setToPositionFromShoulder
    // @param new_foot_pos:       Foot position (in mm) relative to BODY origin, in BODY frame
    // @param time:               Total movement time desired to move the foot from its current position to the new position.
    //                            Set to 0/TIME_INSTANT for max speed.
    void moveToPositionFromBody(Point new_foot_pos, float time=TIME_INSTANT) {
        moveToPositionFromShoulder(new_foot_pos - mounting_pos, time); // may not be accurate
    }

    void adjustLegMotionGoal(Point new_foot_pos) {
    	trajectory_oS.adjustFinalState(new_foot_pos);
    }

    void adjustLegMotionTime(float time) {
    	trajectory_oS.adjustFinalTime(time);
    }

    void moveToDefaultPosition() {
    	moveToPositionFromShoulder(default_foot_position_oS, TIME_INSTANT);
    }

// Accessors
    float getLegSpeed() {
    	float distance_left = (trajectory_oS.getRemainingState(set_foot_position)).norm();
    	return distance_left/trajectory_oS.getRemainingTime();
    }
// 

// ==============================
// ====== UPDATE =======
// ==============================

    // Calculates where the foot position should be based on a trajectory
    // Place in the loop()
    // If no trajectory, does nothing
    // Implementation: Given trajectory, calculates candidate foot information based on time
    // @return Whether new signals should be sent to the servos
    void solveMotion() {  
        if (trajectory_oS.isActive()) { 
 			Point next_set_foot_position = trajectory_oS.getNextState(set_foot_position);
        	next_foot_kinematics = inverseKinematics(next_set_foot_position);
        }   
    }

    bool kinematicsIsValid() {
    	return next_foot_kinematics.is_valid;
    }

    // Sends saved angles to servos. Only use if solveMotion returns true. 
    // Updates appropriate variables
    void sendSignals() {
        servo_driver->gotoAngle(chest_chan, next_foot_kinematics.chest_angle);
        servo_driver->gotoAngle(shoulder_chan, next_foot_kinematics.shoulder_angle);
        servo_driver->gotoAngle(elbow_chan, next_foot_kinematics.elbow_angle);

        // Set state update
        set_foot_position = next_foot_kinematics.foot_pos;
        // Housekeeping
        next_foot_kinematics.reset();
    }

    // Place this in loop if leg is operating independently
    void operate() {
    	if (isInTrajectory()) {
	    	solveMotion();
	    	if (kinematicsIsValid())
		    	sendSignals();
		    else {
		    	endTrajectory();
		    }
		}
    }

    // Cancels a trajectory if dog legs not in sync
    void endTrajectory() {
        trajectory_oS.end();
        next_foot_kinematics.reset();
    }

    bool isInTrajectory() {
        return trajectory_oS.isActive();
    }


    // Debugging
    #ifdef DEBUG
    void printIkinAngles() {
    	Serial.print("IKIN Angles: C: "); Serial.print(next_foot_kinematics.chest_angle);
    	Serial.print(" S: "); Serial.print(next_foot_kinematics.shoulder_angle);
    	Serial.print(" E: "); Serial.print(next_foot_kinematics.elbow_angle);
    	Serial.println();
    }

    void gotoAngles(float chest, float shoulder, float elbow) {
        servo_driver->gotoAngle(chest_chan, chest);
        servo_driver->gotoAngle(shoulder_chan, shoulder);
        servo_driver->gotoAngle(elbow_chan, elbow);
    }
	#endif

};

#endif

// Fixed Leg Processing
       // if (isFixed()) {
       //      if (!centroid_oBfG) {return false;}

       //      // Attempts to 1. Reach goal position 2. Account for displacements caused by body movement
       //      // First, obtain current leg position relative to centroid
       //      // Overall trajectory accounting for body movement error
       //      // PRINT_POINT("current foot (oCfG): ", getCurrentFootPositionFromCentroid(Frame::GROUND))
       //      // PRINT_POINT("goal oCfG: ", goal_foot_pos_of)
       //      Point total_trajectory_fG = goal_foot_pos_of - getCurrentFootPositionFromCentroid(Frame::GROUND); 
       //      // PRINT_POINT("total traj (oCfG): ", total_trajectory_fG)

       //      // Error in trajectory unaccounted for by original trajectory
       //      Point tracking_error_fG = total_trajectory_fG - trajectory_f; 
       //      // PRINT_POINT("track error: ", tracking_error_fG)

       //      // First account for tracking error
       //      Point candidate_set_foot_position_oCfG = goal_foot_pos_of - tracking_error_fG * 0.0;

       //      // Check if trajectory exists (currently in a trajectory)
       //      if (trajectory_f) {
       //          // Serial.println("Traj exists");
       //          // Align main trajectory with drifted error
       //          trajectory_f = total_trajectory_fG.unit() * trajectory_f.norm(); 

       //          // Fraction of trajectory travelled. 0 to 1. 1 means complete
       //          float expected_progress; 
       //          if (traj_total_time == TIME_INSTANT) { // Trajectory is instantaneous
       //              expected_progress = 1;
       //          } else {
       //              expected_progress = (millis() - traj_start_time)/traj_total_time;
       //          }

       //          Serial.print(expected_progress); Serial.print(", ");

       //          // Trajectory is finished. Reset.
       //          if (expected_progress >= 1) {
       //              expected_progress = 1.0;
       //              trajectory_f = POINT_ZERO;

       //          }

       //          // PRINT_POINT("trajectory (g): ", trajectory_f)

       //          // Account for main trajectory
       //          candidate_set_foot_position_oCfG -= trajectory_f * (1 - expected_progress);
       //      }

       //      // Convert to body frame, from shoulder
       //      foot_candidate_info.foot_pos = (candidate_set_foot_position_oCfG + (*centroid_oBfG)) * (*body_orientation) - mounting_pos;
       //      // PRINT_POINT("Candidate: ", foot_candidate_info.foot_pos)
       //      return inverseKinematics(foot_candidate_info.foot_pos);