#ifndef __DOGLEG__
#define __DOGLEG__

#include <math.h>
#include <math2.h>
#include <arduino.h>
#include "RServoDriver.h"
#include <Point.h>
#include <Rot.h>

    #define PRINT_POINT(a, b) Serial.print(a); b.print();

 // #define DEBUG

// Dog leg on a body

// USAGE DETAILS
// Calculates inverse kinematics to move foot to a desired position.
// Desired foot position can be:
//  - Relative to the shoulder
//  - Relative to the body origin
//  - In ground or body frame
//
// COORDINATE FRAME
// Reference Leg (top view):
//  -------
//  |  o  |  front
//  ------*
//   right    
// o: body origin. *: shoulder origin
//
// Reference frame of COM (side view):
//   ^ +z (up)
//   |
//   . --> +x (front)
//   +y (right)
//
// - Mounting Position: Distance to shoulder from body center
//
// COORDINATION STATE
// - Leg may be set to various coordination states:
//   Fixed: Foot trajectory is constant in ground frame regardless of body orientation
//   Floating: Foot trajectory is constant in body frame
//   Anchored: Foot position is constant in ground frame, and foot is in contact with the ground/a surface
// - For consistency with coordination states, requesting a movement in a given frame...creates
//   a trajectory dependent on the coordination state
//
// ORIENTATION
// - The body orientation refers to the CURRENT orientation, not the desired one.
//   It is primarily used for - accessing the current foot position
//                            - locating the ground frame
//   Obtaining a desired orientation is the responsibility of the dog class.

// TODO:
// - Take into account the dog's current position from the centroid.
// - If motion is requested and trajectory already exists, add to old trajectory.

/**
* IMPLEMENTATION DETAILS
* 
* Chest: servo about x axis
* Shoulder: servo about y axis; upper arm
* Elbow: servo parallel to y axis; forearm
* 
* - Trajectory: Allows speed specification.
*   Fixed speed in trajectory. Not polynomial, because already using a servo
* 
* - Convert ground to body frame: p_B = p_G * orientation
* - Convert body to ground frame: p_G = p_B / (orientation)
* 
* - All points are in BODY FRAME and RELATIVE TO SHOULDER ORIGIN unless otherwise noted
*      - Some exceptions are noted below
* 
* - To reduce computational requirements, foot positions (listed) are stored differenely based on 
*   the coordination state.
*      Trajectory
*      Goal foot position
*   If foot is FIXED, positions are stored in GROUND frame, CENTROID origin
*   If foot is FLOATING or ANCHORED, positions are stored in BODY frame, SHOULDER origin
*
* - Inverse Kinematics ALWAYS performed in BODY frame, relative to SHOULDER
*/


enum class Frame {GROUND, BODY};
enum class OriginReference{BODY, SHOULDER};
enum class CoordinationState {ANCHORED, FIXED, FLOATING};

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

    #define UPDATE_RATE 5 // ms
    #define TRACKING_SPEED 40 // mm/s Speed at which fixed leg attempts to correct itself
    #define MAX_SPEED 140.0 // mm/s


    // ==================================================
    // ============= Signals ============================
    // ==================================================
	int elbow_chan; // lower joint
	int shoulder_chan; // upper joint. reversed to elbow
	int chest_chan; // "roll"

    RServoDriver *servo_driver;

    // ==================================================
    // ============= Parameters =========================
    // ==================================================
	double default_length;
    Point mounting_pos; // Mounting point relative to body origin IN BODY FRAME
    Point foot_pos_default; // in BODY frame, relative to shoulder

    // ==================================================
    // ============= State ==============================
    // ==================================================
    Rot *body_orientation;  // The orientation that the body is CURRENTLY in
    Point *centroid_oBfG;
    Point cur_foot_pos; // Position relative to the shoulder origin. BODY frame 

    // ==================================================
    // ============= Coordination =======================
    // ==================================================
    // Holds a foot position and associated servo angles from IKin
    // Body frame, relative to shoulder
    struct ikin_info {
        Point foot_pos; // Body frame, from shoulder
    	double chest_angle;
	    double shoulder_angle;
	    double elbow_angle;

	    ikin_info() {
            foot_pos = POINT_ZERO;
	    	chest_angle = 0;
	    	shoulder_angle = 0;
	    	elbow_angle = 0;
	    }
	    ikin_info(Point new_foot_pos, double c, double s, double e) {
            foot_pos = new_foot_pos;
	    	chest_angle = c;
	    	shoulder_angle = s;
	    	elbow_angle = e;
	    }
    };

    ikin_info foot_candidate_info; // IKIN info for leg to check against other legs. shoulder origin, body frame

    CoordinationState coord_state;  // fixed, anchored, floating
                                    // Affects how foot positions and trajectory are stored:
                                    /* Floating: Points stored in BODY frame
                                     * Anchored: Points stored in BODY frame
                                     * Fixed: Points stored in GROUND frame. Goal position stored from CENTROID
                                     */

    // ==================================================
    // ============= Trajectory =========================
    // ==================================================
    Point trajectory_f;       // Path vector from the starting position to the goal position. Set to POINT_ZERO if not currently in trajectory. 
                            // GROUND frame if foot is FIXED. BODY frame if foot is FLOATING
    Point goal_foot_pos_of;    // Foot position at the end of the trajectory. GROUND frame if foot is FIXED. BODY frame if foot is FLOATING
    double traj_start_time; // Starting time of the trajectory
    double traj_total_time; // Total time of the trajectory, in ms

public:
    DogLeg() {}


    DogLeg(RServoDriver *ndriver, int p1, int p2, int p3, bool flip_lr, bool flip_fb, Rot *body_orientation_ref=NULL,
           Point mounting_point=POINT_ZERO, double starting_height=L_DEFAULT_HEIGHT) {
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
        }

        // Save orientation
        body_orientation = body_orientation_ref;
        centroid_oBfG = NULL;

        // Default Startup
        coord_state = CoordinationState::ANCHORED;

        mounting_pos = mounting_point;
        foot_pos_default = Point(0, 0, -starting_height);
        // setToPositionFromShoulder(foot_pos_default, Frame::BODY, 1000);
        foot_candidate_info.foot_pos = foot_pos_default;
        inverseKinematics(foot_pos_default);

        // cur_speed = DEFAULT_SPEED;
        cur_foot_pos = foot_pos_default;

        trajectory_f = POINT_ZERO;
        goal_foot_pos_of = foot_pos_default;
        traj_start_time = 0; 
        traj_total_time = 0; 
    }

      // Additional offsets to servo angles
    void calibrateServos(double off1, double off2, double off3) {
        servo_driver->setOffset(elbow_chan, ELBOW_OFFSET_MAIN + off3);
        servo_driver->setOffset(shoulder_chan, SHOULDER_OFFSET_MAIN + off2);
        servo_driver->setOffset(chest_chan, CHEST_OFFSET_MAIN + off1);
    }

    void setCentroidRef(Point *centroid_ref) {
        centroid_oBfG = centroid_ref;
    }

    // void gotoAngles(double chest, double shoulder, double elbow) {
    // 	angles_candidate = servo_angles(chest, shoulder, elbow);
    // 	sendSignals();
    // }

    Point getCurrentFootPositionFromShoulder(Frame frame) {
        if (frame == Frame::GROUND) {
            return (cur_foot_pos) / (*body_orientation);
        } else { // frame == Frame::BODY
            return cur_foot_pos;
        } 
    }

    Point getCurrentFootPositionFromBody(Frame frame) {
        if (frame == Frame::GROUND) {
            return (cur_foot_pos + mounting_pos) / (*body_orientation);
        } else { // frame == Frame::BODY
            return cur_foot_pos + mounting_pos;
        } 
    }

    Point getCurrentFootPositionFromCentroid(Frame frame) {
        //if (!centroid_oBfG || !isFixed()) return POINT_ZERO;

        if (frame == Frame::GROUND) {
            return getCurrentFootPositionFromBody(Frame::GROUND) - (*centroid_oBfG);
        } else { // frame == Frame::BODY
            return getCurrentFootPositionFromBody(Frame::BODY) - (*centroid_oBfG) * (*body_orientation);
        } 
    }

// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ======== COORDINATION STATE ============~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
public:
    bool isAnchored() {
        return (coord_state == CoordinationState::ANCHORED);
    }

    bool isFixed() {
        return (coord_state == CoordinationState::FIXED);
    }

    bool isFloating() {
        return coord_state == CoordinationState::FLOATING;
    }

    // Swaps state. Also makes sure points are stored in the correct frame.
    // Implementation: Frame switches are not necessary in most contexts. It is mostly only useful if
    //                 switching states in the middle of a trajectory.
    void setState(CoordinationState new_state) {
        // Switch from floating or anchored -> fixed
        if (coord_state != CoordinationState::FIXED && new_state == CoordinationState::FIXED) {
            if (!centroid_oBfG) return; // Fixed frame only possible if centroid is known/available 
            // Points originally in BODY frame. Change to GROUND frame
            // PRINT_POINT("goal orig: ", goal_foot_pos_of)
            trajectory_f      = POINT_ZERO;
            goal_foot_pos_of   = (goal_foot_pos_of + mounting_pos)/(*body_orientation) - (*centroid_oBfG);
            // PRINT_POINT("centroid: ", (*centroid_oBfG))
            // PRINT_POINT("goal new: ", goal_foot_pos_of)
        }
        // Switch from fixed -> floating or anchored
        else if (coord_state == CoordinationState::FIXED && new_state != CoordinationState::FIXED) {
            // Points originally in GROUND frame. Change to BODY frame
            trajectory_f      = POINT_ZERO;
            goal_foot_pos_of   = (goal_foot_pos_of + (*centroid_oBfG)) * (*body_orientation);
        }
        
        coord_state = new_state;
    }

// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ======== INVERSE KINEMATICS ============~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
private:
  	// Moves to the requested position RELATIVE TO MOUNTING POINT, IN BODY FRAME
  	bool inverseKinematics(double x, double y, double z) {
        // x_ and z_ are the x and z coordinates in the reference plane coincident with the arm plane
        double x_ = x;
        double z_ = sqrt(y*y + z*z);

        // Converts values to state
        double l_arm_eff = sqrt(x_*x_ + z_*z_);
        double angle_shoulder_eff = atan2(x_, z_) RAD;
        double angle_chest = -atan2(y, -z) RAD;

  	    // Solve for shoulder, elbow angles
        // Angle from forearm to upper arm. 
        double angle_elbow_relative;
        // Angle of shoulder
        double angle_shoulder;
        
        if (L_FOREARM == L_UPPERARM) { // Optimization if arm lengths are the same
            angle_elbow_relative = 2*asin(l_arm_eff / (2 * L_FOREARM)) RAD;
            angle_shoulder = angle_shoulder_eff - (180.0 - angle_elbow_relative) * 0.5;
        } else {
            // Note: acos is always positive, so joint never inverts!
            angle_elbow_relative = acos((L_FOREARM*L_FOREARM + L_UPPERARM*L_UPPERARM - l_arm_eff*l_arm_eff) /(2 * L_FOREARM * L_UPPERARM)) RAD; 
            angle_shoulder = angle_shoulder_eff - asin(L_FOREARM / l_arm_eff * sin(angle_elbow_relative DEG)) RAD;
        }

        // Angle of elbow
        double angle_elbow = 180 + angle_shoulder - angle_elbow_relative;
        angle_elbow -= WISHBONE_ANGLE; // To get servo angle

        // Save candidate info
        if (WITHIN(angle_chest, -75, 75) && 
            WITHIN(angle_elbow, -185, -67) && 
            WITHIN(angle_shoulder - angle_elbow, 20, 160)) {

            foot_candidate_info.chest_angle = angle_chest;
            foot_candidate_info.shoulder_angle = angle_shoulder;
            foot_candidate_info.elbow_angle = angle_elbow;
            return true;
        } else {
            return false;
        }
  	}

    // Moves foot to a point in the BODY frame relative to the SHOULDER position.
    bool inverseKinematics(Point p) {
    	return inverseKinematics(p.x, p.y, p.z);
    }

// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ======== MOVEMENT COMMANDS =============~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ========================================~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
public:
    // USE THESE TO MOVE ON DOG BODY

    // Moves the foot to the position in the requested frame, relative to the SHOULDER origin
    // Implementation: Set up trajectory information so that operate() can move the foot
    // @param new_foot_pos:       Foot position (in mm) relative to shoulder origin, in requested frame
    // @param frame:              Reference frame in which new_foot_pos is given.
    // @param time:               Total movement time desired to move the foot from its current position to the new position.
    //                            Set to 0/TIME_INSTANT for max speed.
    void setToPositionFromShoulder(Point new_foot_pos_f, Frame frame=Frame::BODY,  double time=TIME_INSTANT) {
        // TODO: account for fixed goal being from centroid
        // Set up trajectory info based on leg state
        if ((frame == Frame::GROUND)) {
            if (!body_orientation) return; // Ensures ground frame is locatable

            if (isFixed()) { // is FIXED. Store in ground frame
                goal_foot_pos_of = new_foot_pos_f;
                trajectory_f = goal_foot_pos_of - cur_foot_pos / (*body_orientation);
            } else { // is ANCHORED or FLOATING. Store in body frame
                goal_foot_pos_of = new_foot_pos_f * (*body_orientation);
                trajectory_f = goal_foot_pos_of - cur_foot_pos;
            }    

        } else { // frame == Frame::BODY
           if (isFixed()) { // is FIXED. Store in ground frame
                goal_foot_pos_of = new_foot_pos_f / (*body_orientation);
                trajectory_f = goal_foot_pos_of - cur_foot_pos / (*body_orientation);
            } else { // is ANCHORED or FLOATING. Store in body frame
                goal_foot_pos_of = new_foot_pos_f;
                trajectory_f = goal_foot_pos_of - cur_foot_pos;
            }  
        }

        traj_start_time = millis();
        traj_total_time = max(time, trajectory_f.norm() / (MAX_SPEED/1000.0));
    }

    // Moves the foot to the position in the requested frame, relative to the BODY origin
    // Implementation: Sets up information to pass to setToPositionFromShoulder
    // @param new_foot_pos:       Foot position (in mm) relative to shoulder origin, in requested frame
    // @param frame:              Reference frame in which new_foot_pos is given.
    // @param time:               Total movement time desired to move the foot from its current position to the new position.
    //                            Set to 0/TIME_INSTANT for max speed.
    void setToPositionFromBody(Point new_foot_pos_oBf, Frame frame=Frame::BODY, double time=TIME_INSTANT) {
        Point new_foot_pos_f;
        if ((frame == Frame::GROUND)) {
            if (!body_orientation) return; // Ensures ground frame is locatable
            new_foot_pos_f = new_foot_pos_oBf - (mounting_pos / (*body_orientation)); // In ground frame
        } else {
            new_foot_pos_f = new_foot_pos_oBf - mounting_pos; // In body frame
        }

        setToPositionFromShoulder(new_foot_pos_f, frame, time); // may not be accurate
    }

    // Need to make sure only fixed leg can access?
    void setToPositionFromCentroid(Point new_foot_pos_oCf, Frame frame=Frame::BODY, double time=TIME_INSTANT) {
        Point new_foot_pos_oBf;
        if ((frame == Frame::GROUND)) {
            if (!body_orientation) return; // Ensures ground frame is locatable
            new_foot_pos_oBf = new_foot_pos_oCf; // In ground frame
        } else {
            new_foot_pos_oBf = new_foot_pos_oCf / (*body_orientation); // In body frame
        }
        goal_foot_pos_of = new_foot_pos_oBf;
        trajectory_f = goal_foot_pos_of - getCurrentFootPositionFromCentroid(Frame::GROUND);
        traj_start_time = millis();
        traj_total_time = max(time, trajectory_f.norm() / (MAX_SPEED/1000.0));;

    }

    // Moves the foot by the amount relative to its original position
    // Implementation: Sets up information to pass to setToPositionFromShoulder
    void setFromPosition(Point d_foot_pos, Frame frame=Frame::BODY, double time=TIME_INSTANT) {
        Point new_foot_pos;
        if ((frame == Frame::GROUND)) {
            if (!body_orientation) return; // Ensures ground frame is locatable
            new_foot_pos = cur_foot_pos / (*body_orientation) + d_foot_pos; // In ground frame
        } else {
            new_foot_pos = cur_foot_pos + d_foot_pos; // In body frame
        }

      	setToPositionFromShoulder(new_foot_pos, frame, time);
    }

    // Calculates where the foot position should be based on a trajectory
    // Place in the loop()
    // If no trajectory (trajectory == POINT_ZERO), does nothing
    // Implementation: Given trajectory, calculates candidate foot information based on time
    // @return Whether IKIN failed or not
    bool operate() {
        if (isFixed()) {
            if (!centroid_oBfG) {return false;}

            // Attempts to 1. Reach goal position 2. Account for displacements caused by body movement
            // First, obtain current leg position relative to centroid
            // Overall trajectory accounting for body movement error
            // PRINT_POINT("current foot (oCfG): ", getCurrentFootPositionFromCentroid(Frame::GROUND))
            // PRINT_POINT("goal oCfG: ", goal_foot_pos_of)
            Point total_trajectory_fG = goal_foot_pos_of - getCurrentFootPositionFromCentroid(Frame::GROUND); 
            // PRINT_POINT("total traj (oCfG): ", total_trajectory_fG)

            // Error in trajectory unaccounted for by original trajectory
            Point tracking_error_fG = total_trajectory_fG - trajectory_f; 
            // PRINT_POINT("track error: ", tracking_error_fG)

            // First account for tracking error
            Point candidate_foot_position_oCfG = goal_foot_pos_of - tracking_error_fG * 0.1;

            // Check if trajectory exists
            if (trajectory_f) {
                // Serial.println("Traj exists");
                // Align main trajectory with drifted error
                trajectory_f = total_trajectory_fG.unit() * trajectory_f.norm(); 

                // Percentage of trajectory travelled
                double expected_progress; 
                if (traj_total_time == TIME_INSTANT) { // Trajectory is instantaneous
                    expected_progress = 1;
                } else {
                    expected_progress = (millis() - traj_start_time)/traj_total_time;
                }

                // Trajectory is finished. Reset.
                if (expected_progress >= 1) {
                    expected_progress = 1.0;
                    trajectory_f = POINT_ZERO;
                }

                // PRINT_POINT("trajectory (g): ", trajectory_f)

                // Account for main trajectory
                candidate_foot_position_oCfG -= trajectory_f * (1 - expected_progress);
            }

            // Convert to body frame, from shoulder
            foot_candidate_info.foot_pos = (candidate_foot_position_oCfG + (*centroid_oBfG)) * (*body_orientation) - mounting_pos;
            // PRINT_POINT("Candidate: ", foot_candidate_info.foot_pos)
            return inverseKinematics(foot_candidate_info.foot_pos);
        } else { // FLOATING OR ANCHORED
            // Check if currently in a trajectory
            if (trajectory_f) {
                // Percentage of trajectory travelled
                double expected_progress; 
                if (traj_total_time == TIME_INSTANT) { // Trajectory is instantaneous
                    expected_progress = 1;
                } else {
                    expected_progress = (millis() - traj_start_time)/traj_total_time;
                }

                // Trajectory is finished. Reset.
                if (expected_progress >= 1) {
                    expected_progress = 1.0;
                    trajectory_f = POINT_ZERO;
                }

                // PRINT_POINT("goal pos (oSfB): ", goal_foot_pos_of);

                // Calculates the appropriate leg position in the BODY frame 
                foot_candidate_info.foot_pos = goal_foot_pos_of - trajectory_f * (1 - expected_progress);
                // and passes to IKIN
                return inverseKinematics(foot_candidate_info.foot_pos);
            }
        }

        return true;
    }

    // Sends saved angles to servos
    void sendSignals() {
        cur_foot_pos = foot_candidate_info.foot_pos;
        servo_driver->gotoAngle(chest_chan, foot_candidate_info.chest_angle);
        servo_driver->gotoAngle(shoulder_chan, foot_candidate_info.shoulder_angle);
        servo_driver->gotoAngle(elbow_chan, foot_candidate_info.elbow_angle);
    }

    // Cancels a trajectory if dog legs not in sync
    void cancelTrajectory() {
        trajectory_f = POINT_ZERO;
    }

    bool isIdle() {
        return !trajectory_f;
    }
};

#endif


    // Checks that the coordination state is consistent with the given frame
    // If not, fixes it and prints an error message
    // bool patchCoordinationState(Frame frame) {
    //     if (frame == Frame::GROUND) {
    //         if (isFloating()) {
    //             coord_state = CoordinationState::FIXED;

    //             // Points originally in BODY frame. Change to GROUND frame
    //             trajectory      /= (*body_orientation); // Do these need to be fixed?
    //             goal_foot_pos_of   /= (*body_orientation); //
    //             //cur_foot_pos    /= (*body_orientation);

    //             Serial.println("NOTICE: Coordination state changed from FLOATING to FIXED");
    //             return false;
    //         }
    //     } else { // frame == Frame::BODY
    //         if (!isFloating()) {
    //             coord_state = CoordinationState::FLOATING;

    //             // Points originally in GROUND frame. Change to BODY frame
    //             trajectory      *= (*body_orientation);
    //             goal_foot_pos_of   *= (*body_orientation);
    //             //cur_foot_pos    *= (*body_orientation);

    //             Serial.println("NOTICE: Coordination state changed from FIXED/ANCHORED to FLOATING");
    //             return false;
    //         }
    //     }
    //     return true;
    // }


    // // Moves the foot to the position in the requested frame, relative to the SHOULDER origin
    // // Implementation: Set up trajectory_f information so that operate() can move the foot
    // // @param new_foot_pos:       Foot position (in mm) relative to shoulder origin, in requested frame
    // // @param frame:              Reference frame in which new_foot_pos is given. GROUND or BODY
    // // @param body_orientation:   Self-explanatory. Used only when frame == GROUND
    // // @param time:               Total movement time desired to move the foot from its current position to the new position.
    // //                            Set to 0/TIME_INSTANT for max speed.
    // // deprecated @param speed:              speed (mm/s) of movement. Set to 0 for "instantaneous" movement/max speed
    // bool gotoPositionFromShoulder(Point new_foot_pos, Frame frame, Rot body_orientation=ROT_ZERO,  double time=TIME_INSTANT) {
    //     // Calculate goal position based on frame
    //     if (frame == BODY)
    //         goal_foot_pos_of = new_foot_pos;
    //     else
    //         goal_foot_pos_of = new_foot_pos.inv_rotate(body_orientation);

    //     // Set up trajectory info
    //     if (time != TIME_INSTANT) { // If speed given, set trajectory at a given speed
    //         traj_start_time = millis()/1000.0;
    //         // cur_speed = speed;
    //         // traj_total_time = trajectory.norm() / speed;
    //         trajectory = goal_foot_pos_of - cur_foot_pos;
    //         traj_total_time - time;
    //     } else { // If time is 0 ("instantaneous"), bypasses trajectory and sets up candidate information
    //         foot_candidate_info.foot_pos = goal_foot_pos_of;
    //         Serial.println("instant move");
    //         return inverseKinematics(foot_candidate_info.foot_pos);
    //     }
    //     return true;
    // }