#ifndef __DOGLEG__
#define __DOGLEG__

#include <math.h>
#include <math2.h>
#include <arduino.h>
#include "RServoDriver.h"
#include <Point.h>
#include <Rot.h>
#include "Trajectory.h"


   

 // #define DEBUG


// TODO: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// - Trajectory Adjustment
// - Incorporate default position into startup
// - Where to place solved kinematics for next foot position?
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
    #define SHOULDER_OFFSET_MAIN -15
    #define ELBOW_OFFSET_MAIN -125

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

    // ============= Kinematics =======================
    // Holds a foot position and associated servo angles from IKin
    // Body frame, relative to shoulder
    struct IkinInfo {
        Point foot_pos; // Body frame, from shoulder
    	float chest_angle;
	    float shoulder_angle;
	    float elbow_angle;
	    bool is_valid;

	    IkinInfo() = default;

	    IkinInfo(Point new_foot_pos, float c, float s, float e) {
            foot_pos = new_foot_pos;
	    	chest_angle = c;
	    	shoulder_angle = s;
	    	elbow_angle = e;
	    }

	    void reset() {
            foot_pos = POINT_ZERO;
	    	chest_angle = 0;
	    	shoulder_angle = 0;
	    	elbow_angle = 0;
	    	is_valid = false;
	    }
    };

    IkinInfo foot_candidate_info; // IKIN info for a working leg position. transient. shoulder origin, body frame

    // ============= Trajectory =========================
    TrajectoryInfo leg_trajectory_oS;

    // =========== Parameters ====================
    float L_CHEST;


    // ==================================================
    // ============= Accessed ===========================
    // ==================================================
    Point mounting_pos; // Mounting point relative to body origin IN BODY FRAME
    // $$ Manually Set
    Point set_foot_position; // Position relative to the shoulder origin. BODY frame 

public:
    DogLeg() {}


    DogLeg(RServoDriver *ndriver, int p1, int p2, int p3, Point mounting_point=POINT_ZERO, float starting_height=L_DEFAULT_HEIGHT) {
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

        if (flip_fb) {
        	servo_driver->reverseDirection(chest_chan);
        } 
        if (flip_lr) {
        	servo_driver->reverseDirection(elbow_chan);
        	servo_driver->reverseDirection(shoulder_chan);
        	L_CHEST *= -1;
        }

        mounting_pos = mounting_point;
        foot_pos_default = Point(0, 0, -starting_height);
        // setToPositionFromShoulder(foot_pos_default, Frame::BODY, 1000);
        foot_candidate_info.foot_pos = foot_pos_default;
        inverseKinematics(foot_pos_default);

        set_foot_position = foot_pos_default;

        trajectory = LegTrajectoryInfo();
    }

    void flipLR() {
    	servo_driver->reverseDirection(chest_chan);
    }

    void flipFB() {
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

    // void gotoAngles(float chest, float shoulder, float elbow) {
    // 	angles_candidate = servo_angles(chest, shoulder, elbow);
    // 	sendSignals();
    // }

    Point getFootPosition_oBfB() {
        return set_foot_position + mounting_pos;
    }

    Point getMountingPoint() {
    	return mounting_pos;
    }

// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ======== INVERSE KINEMATICS ============~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
private:
	IkinInfo inverseKinematics(float x, float y, float z) {
		IkinInfo kinematics_info;
        // x_ and z_ are the x and z coordinates in the reference plane coincident with the arm plane
        
        // // Impossible position check
        // if ((y*y + z*z - L_CHEST*L_CHEST) < 0) 	return false;
        float x_ = x;
        float z_ = sqrt(y*y + z*z - L_CHEST*L_CHEST);

        // Converts values to state
        float l_arm_eff = sqrt(x_*x_ + z_*z_);
        float angle_shoulder_eff = atan2(x_, z_) RAD;
        float angle_chest = 90 - (atan2(z_, L_CHEST) RAD - atan2(-y, -z) RAD);

  	    // Solve for shoulder, elbow angles
        // Angle from forearm to upper arm. 
        float angle_elbow_relative;
        // Angle of shoulder
        float angle_shoulder;
        
        if (L_FOREARM == L_UPPERARM) { // Optimization if arm lengths are the same
            angle_elbow_relative = 2*asin(l_arm_eff / (2 * L_FOREARM)) RAD;
            angle_shoulder = angle_shoulder_eff - (180.0 - angle_elbow_relative) * 0.5;
        } else {
            // Note: acos is always positive, so joint never inverts!
            angle_elbow_relative = acos((L_FOREARM*L_FOREARM + L_UPPERARM*L_UPPERARM - l_arm_eff*l_arm_eff) /(2 * L_FOREARM * L_UPPERARM)) RAD; 
            angle_shoulder = angle_shoulder_eff - asin(L_FOREARM / l_arm_eff * sin(angle_elbow_relative DEG)) RAD;
        }

        // Angle of elbow
        float angle_elbow = 180 + angle_shoulder - angle_elbow_relative;
        angle_elbow -= WISHBONE_ANGLE; // To get servo angle

        // Check if returned angles are feasible
        bool ikin_err = false;
        if (!WITHIN(angle_chest, -75, 75)) 					{Serial.print("IKIN ERR: Chest at "); 			 Serial.println(angle_chest); 					ikin_err = true;}
        if (!WITHIN(angle_elbow, -185, -67)) 				{Serial.print("IKIN ERR: Elbow at "); 			 Serial.println(angle_elbow); 					ikin_err = true;}
        if (!WITHIN(angle_shoulder - angle_elbow, 20, 160)) {Serial.print("IKIN ERR: Shoulder - Elbow at "); Serial.println(angle_shoulder - angle_elbow);  ikin_err = true;}
        
        // Saves the calculated values (as appropriate)
        if (ikin_err) {
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
    bool inverseKinematics(Point p) {
    	return inverseKinematics(p.x, p.y, p.z);
    }


public:
// ==============================
// ====== MOTION COMMANDS =======
// ==============================

    // Moves the foot to the position in the requested frame, relative to the SHOULDER origin
    // Implementation: Set up trajectory information so that operate() can move the foot
    // @param new_foot_pos:       Foot position (in mm) relative to SHOULDER origin, in BODY frame
    // @param time:               Total movement time desired to move the foot from its current position to the new position.
    //                            Set to 0/TIME_INSTANT for max speed.
    void moveLegToPositionFromShoulder(Point new_foot_pos, float time=TIME_INSTANT) {       
        // Set up trajectory info based on leg state
        leg_trajectory_oS.begin(new_foot_pos, time)
    }

    // Moves the foot to the position in the requested frame, relative to the BODY origin
    // Implementation: Sets up information to pass to setToPositionFromShoulder
    // @param new_foot_pos:       Foot position (in mm) relative to BODY origin, in BODY frame
    // @param time:               Total movement time desired to move the foot from its current position to the new position.
    //                            Set to 0/TIME_INSTANT for max speed.
    void moveLegToPositionFromBody(Point new_foot_pos, float time=TIME_INSTANT) {
        moveLegToPositionFromShoulder(new_foot_pos - mounting_pos, time); // may not be accurate
    }

// ==============================
// ====== OPERATIONAL COMMANDS =======
// ==============================

    // Calculates where the foot position should be based on a trajectory
    // Place in the loop()
    // If no trajectory, does nothing
    // Implementation: Given trajectory, calculates candidate foot information based on time
    // @return Whether new signals should be sent to the servos
    bool solveMotion() {  
         // No motion to solve if not in trajectory
        if (!leg_trajectory_oS.isActive()) { return false;}

    	// Calculate the next appropriate foot position
    	Point next_set_foot_position = leg_trajectory_oS.getNextState(set_foot_position);

        // then passes to IKIN to solve.
        // The result is either the kinematics or a reported error
        trajectory.next_set_foot_position_kinematics = inverseKinematics(next_set_foot_position);
        return trajectory.next_set_foot_position_kinematics.is_valid;
    }

    // Sends saved angles to servos. Only use if solveMotion returns true. 
    // Updates appropriate variables
    void sendSignals() {
    	// Sends the signals
    	IkinInfo kinematics_info = trajectory.next_set_foot_position_kinematics;
        servo_driver->gotoAngle(chest_chan, kinematics_info.chest_angle);
        servo_driver->gotoAngle(shoulder_chan, kinematics_info.shoulder_angle);
        servo_driver->gotoAngle(elbow_chan, kinematics_info.elbow_angle);

        // Set state update
        set_foot_position = kinematics_info.foot_pos;
        // Housekeeping
        trajectory.next_set_foot_position_kinematics.reset();
    }

    // Place this in loop if leg is operating independently
    void operate() {
    	if (isInTrajectory()) {
	    	if (solveMotion()) 
		    	sendSignals();
		    else
		    	trajectory.end();
		}
    }

    // Cancels a trajectory if dog legs not in sync
    void endTrajectory() {
        trajectory.end();
    }

    bool isInTrajectory() {
        return trajectory.isActive();
    }


    // Debugging
    void printIkinAngles() {
    	Serial.print("IKIN Angles: C: "); Serial.print(foot_candidate_info.chest_angle);
    	Serial.print("S: "); Serial.print(foot_candidate_info.shoulder_angle);
    	Serial.print("E: "); Serial.print(foot_candidate_info.elbow_angle);
    	Serial.println();
    }
};

#endif

// Ground Frame Conversion
        // if ((frame == Frame::GROUND)) {
        //     if (!body_orientation) return; // Ensures ground frame is locatable
        //     new_foot_pos = current_set_foot_position / (*body_orientation) + d_foot_pos; // In ground frame
        // } else {

    //   Point getCurrentFootPositionFromCentroid(Frame frame) {
    //     //if (!centroid_oBfG || !isFixed()) return POINT_ZERO;

    //     if (frame == Frame::GROUND) {
    //         return getCurrentFootPositionFromBody(Frame::GROUND) - (*centroid_oBfG);
    //     } else { // frame == Frame::BODY
    //         return getCurrentFootPositionFromBody(Frame::BODY) - (*centroid_oBfG) * (*body_orientation);
    //     } 
    // }

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


// Centroid Motion
    // // Need to make sure only fixed leg can access?
    // void setToPositionFromCentroid(Point new_foot_pos_oCf, Frame frame=Frame::BODY, float time=TIME_INSTANT) {
    //     Point new_foot_pos_oBf;
    //     if ((frame == Frame::GROUND)) {
    //         if (!body_orientation) return; // Ensures ground frame is locatable
    //         new_foot_pos_oBf = new_foot_pos_oCf; // In ground frame
    //     } else {
    //         new_foot_pos_oBf = new_foot_pos_oCf / (*body_orientation); // In body frame
    //     }
    //     goal_foot_pos_of = new_foot_pos_oBf;
    //     trajectory_f = goal_foot_pos_of - getCurrentFootPositionFromCentroid(Frame::GROUND);
    //     traj_start_time = millis();

    //     traj_total_time = max(time, trajectory_f.norm() / (MAX_SPEED/1000.0));;
    // }
