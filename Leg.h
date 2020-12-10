#ifndef __DOGLEG__
#define __DOGLEG__

#include <math.h>
#include <math2.h>
#include <arduino.h>
#include "RServoDriver.h"
#include <Point.h>
#include <Rot.h>
#include "Trajectory.h"


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
    #define DEFAULT_SPEED 10 // mm/s


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

    TrajectoryInfo<Point> foot_trajectory_oS;
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
    Point set_foot_position_oS; // Position relative to the shoulder origin. BODY frame 

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
        foot_trajectory_oS = TrajectoryInfo<Point>(default_foot_position_oS);
        foot_trajectory_oS.setSpeed(DEFAULT_SPEED);

        // Accessed
        set_foot_position_oS = default_foot_position_oS;
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


// ==============================
// ====== UPDATE =======
// ==============================
    // Place this in loop if leg is operating independently
    void operate() {
    	if (isInTrajectory()) {
    		solveMotion();
	    	if (kinematicsIsValid())
		    	sendSignalsAndSavePosition();
		    else {
		    	endTrajectory();
		    }
		} else {
			syncTrajectoryTimer();
		}
    }

    // Calculates where the foot position should be based on a trajectory
    // Place in the loop()
    // If no trajectory, does nothing
    // Implementation: Given trajectory, calculates candidate foot information based on time
    // @return Whether new signals should be sent to the servos
    void solveMotion() {  
        if (foot_trajectory_oS.isInMotion()) { 
        	Point next_set_foot_position_oS = foot_trajectory_oS.getNextState(set_foot_position_oS);
        	next_foot_kinematics = inverseKinematics(next_set_foot_position_oS);
        }   
    }

    void syncTrajectoryTimer() {
    	foot_trajectory_oS.syncTimer();
    }

    bool kinematicsIsValid() {
    	return next_foot_kinematics.is_valid;
    }

    // Sends saved angles to servos. Only use if solveMotion returns true. 
    // Updates appropriate variables
    void sendSignalsAndSavePosition() {
        servo_driver->gotoAngle(chest_chan, next_foot_kinematics.chest_angle);
        servo_driver->gotoAngle(shoulder_chan, next_foot_kinematics.shoulder_angle);
        servo_driver->gotoAngle(elbow_chan, next_foot_kinematics.elbow_angle);

        set_foot_position_oS = next_foot_kinematics.foot_pos;
        next_foot_kinematics.reset();
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

        float angle_chest = atan2(y, -z) RAD - atan2(L_CHEST, -z_) RAD;
        // Note: acos is always positive, so joint never inverts!
        float effective_leg_length = sqrt(x_*x_ + z_*z_);
        float effective_leg_angle = -atan2(x_, -z_) RAD;
        float interior_elbow_angle = acos((L_FOREARM*L_FOREARM + L_UPPERARM*L_UPPERARM - effective_leg_length*effective_leg_length) /(2 * L_FOREARM * L_UPPERARM)) RAD; 
        float angle_shoulder = asin(L_FOREARM / effective_leg_length * sin(interior_elbow_angle DEG)) RAD + effective_leg_angle;
        float angle_elbow = -(180 - angle_shoulder - interior_elbow_angle) + WISHBONE_ANGLE; // servo angle is offset from actual elbow by WISHBONE

        int ikin_error_code = 0;
        if      (!WITHIN(angle_chest, -75, 75)) 					{ikin_error_code = 1;}
        else if (!WITHIN(angle_elbow,  67, 185)) 				    {ikin_error_code = 2;}
        else if (!WITHIN(angle_elbow - angle_shoulder, 20, 160))    {ikin_error_code = 3;}
  
        #ifdef DEBUG 
        if      (ikin_error_code == 1) {Serial.print("IKIN ERR: Chest at "); Serial.println(angle_chest);}
        else if (ikin_error_code == 2) {Serial.print("IKIN ERR: Elbow at "); Serial.println(angle_elbow);}
        else if (ikin_error_code == 3) {Serial.print("IKIN ERR: Shoulder - Elbow at ");  Serial.println(angle_shoulder - angle_elbow);} 
        #endif

        // Saves the calculated values (as appropriate)
        if (ikin_error_code == 0) {
            kinematics_info.foot_pos = Point(x, y, z);
            kinematics_info.chest_angle = angle_chest;
            kinematics_info.shoulder_angle = angle_shoulder;
            kinematics_info.elbow_angle = angle_elbow;
            kinematics_info.is_valid = true;
        } else {
            kinematics_info.is_valid = false;

            #ifdef DEBUG
            if (id != -1) {
                Serial.print("Leg "); Serial.print(id); Serial.print(": ");
            }
            Serial.print("Requested Point: "); Point(x, y, z).print();
            #endif
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
    	if (!set_foot_position_oS) return POINT_NULL;
        return set_foot_position_oS + mounting_pos;
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
    void setFootSpeed(float speed) {
    	foot_trajectory_oS.setSpeed(speed);
    }
    // Moves the foot to the position in the requested frame, relative to the SHOULDER origin
    // Implementation: Set up trajectory information so that operate() can move the foot
    // @param new_foot_pos:       Foot position (in mm) relative to SHOULDER origin, in BODY frame
    // @param time:               Total movement time desired to move the foot from its current position to the new position.
    //                            Set to 0/TIME_INSTANT for max speed.
    void moveToPositionFromShoulderAtSpeed(Point new_foot_pos, float speed) {       
        // Set up trajectory info based on leg state
        setFootSpeed(speed);
        moveToPositionFromShoulder(new_foot_pos);
    }

    void moveToPositionFromShoulder(Point new_foot_pos) {     
        foot_trajectory_oS.updateGoal(new_foot_pos);
    }

    // Moves the foot to the position in the requested frame, relative to the BODY origin
    // Implementation: Sets up information to pass to setToPositionFromShoulder
    // @param new_foot_pos:       Foot position (in mm) relative to BODY origin, in BODY frame
    // @param time:               Total movement time desired to move the foot from its current position to the new position.
    //                            Set to 0/TIME_INSTANT for max speed.
    void moveToPositionFromBody(Point new_foot_pos) {
        moveToPositionFromShoulder(new_foot_pos - mounting_pos); 
    }

    void moveToPositionFromBodyAtSpeed(Point new_foot_pos, float speed) {
        moveToPositionFromShoulderAtSpeed(new_foot_pos - mounting_pos, speed); 
    }

    void moveToPositionFromBodyInTime(Point new_foot_pos_oB, float time) {       
        float speed;
        if (time == TIME_INSTANT) {
        	speed = INFINITE_SPEED;
        } else {
        	speed = (new_foot_pos_oB - mounting_pos - set_foot_position_oS).norm()/time; 
        }
		moveToPositionFromBodyAtSpeed(new_foot_pos_oB, speed);
    }

    void moveToPositionFromBodyInstantly(Point new_foot_pos) {
        moveToPositionFromShoulderAtSpeed(new_foot_pos - mounting_pos, INFINITE_SPEED); 
    }

    void moveToDefaultPosition() {
    	moveToPositionFromShoulder(default_foot_position_oS);
    }


    // Cancels a trajectory if dog legs not in sync
    void endTrajectory() {
        foot_trajectory_oS.end();
        next_foot_kinematics.reset();
    }

    bool isInTrajectory() {
        return foot_trajectory_oS.isInMotion();
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