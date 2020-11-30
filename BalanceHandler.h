#ifndef __BALANCE__
#define __BALANCE__

#include "Rot.h"
#include "Dog.h"
#include "Timer.h"

#define DEFAULT_D_GAIN 0.04
#define DEFAULT_I_GAIN 0.015

#define MAX_X_ORIENT 30
#define MAX_Y_ORIENT 30
#define MAX_Z_ORIENT 30
#define MAX_ROTATIONAL_VEL 0.3
// TODO I gain is currently dependent on the mesurement interval...need to make independent

class BalanceHandler {
    float measurement_interval = 3.0/1000;
    Timer measurement_update_timer;

    Rot desired_IMU_orientation = ROT_ZERO;

    Rot IMU_orientation = ROT_ZERO;
    Rot IMU_orientation_error = ROT_ZERO;
    Rot IMU_rot_velocity = ROT_ZERO;
    Rot error_integrator = ROT_ZERO;

    Rot limiting_balance_velocity = Rot(MAX_ROTATIONAL_VEL, MAX_ROTATIONAL_VEL, MAX_ROTATIONAL_VEL);
    Rot limiting_balance_magnitude = Rot(MAX_X_ORIENT, MAX_Y_ORIENT, MAX_Z_ORIENT);

    float I_gain;
    float D_gain;


    RobotDog *dog;
public:
    BalanceHandler() = default;
    BalanceHandler(RobotDog *n_dog) {
        dog = n_dog;
        I_gain = DEFAULT_I_GAIN;
        D_gain = DEFAULT_D_GAIN;
        measurement_update_timer.reset(measurement_interval);
    }

    void setIDGains(float n_I_gain, float n_D_gain) {
        I_gain = n_I_gain;
        D_gain = n_D_gain;
    }

    void setDesiredOrientation(Rot n_desired_IMU_orientation) {
    	desired_IMU_orientation = n_desired_IMU_orientation;
    }

    void operate() {
        if (measurement_update_timer.timeOut()) {
        	IMU_orientation = dog->getBodyIMUOrientation_fG2B();
            IMU_orientation_error = dog->getBodyIMUOrientation_fG2B() - desired_IMU_orientation;
            IMU_rot_velocity = dog->getBodyIMURotVelocity_fG2B();
            error_integrator += IMU_orientation_error;
    
            measurement_update_timer.reset();
        }
    }

    Rot getNextKinematicBalancingOrientation() {
        Rot balancing_orientation = -(IMU_orientation_error - IMU_rot_velocity * D_gain + error_integrator * I_gain);

        //Serial.print(balancing_orientation.x); Serial.print(" ");

        balancing_orientation = limitBalancingOrientationVelocity(balancing_orientation);
        balancing_orientation = limitBalancingOrientationMagnitude(balancing_orientation);

		//Serial.print(balancing_orientation.x); 
		//Serial.println();

        return balancing_orientation;
    }

    void setBalancingVelocityLimit(float velocity_limit) {
    	limiting_balance_velocity = Rot(velocity_limit, velocity_limit, velocity_limit);
    }

    void setBalancingMagnitudeLimits(Rot magnitude_limit) {
    	limiting_balance_magnitude = magnitude_limit;
    }
private:
    Rot limitBalancingOrientationVelocity(Rot balancing_orientation) {
    	Rot kine_orientation = dog->getBodyKinematicOrientation_fF2B();
    	Rot balancing_difference = balancing_orientation - kine_orientation;

    	balancing_difference = limitOrientation(balancing_difference, limiting_balance_velocity);
    	return kine_orientation + balancing_difference;

        // float difference_magnitude = (kine_orientation - balancing_orientation).norm();
        // ///Serial.println(difference_magnitude);
        // if (difference_magnitude > MAX_ROTATIONAL_VEL) {
        // 	Rot d_orientation = (balancing_orientation - kine_orientation) / difference_magnitude * MAX_ROTATIONAL_VEL;
        // 	balancing_orientation = (kine_orientation + d_orientation); 
        // }
    }

    Rot limitBalancingOrientationMagnitude(Rot balancing_orientation) {
    	Rot limiting_orientation = limiting_balance_magnitude + desired_IMU_orientation;
    	return limitOrientation(balancing_orientation, limiting_orientation);
    }

    Rot limitOrientation(Rot orientation, Rot limiting_orientation) {
    	if (fabs(orientation.x) > limiting_orientation.x) {
    		orientation.x = orientation.x/fabs(orientation.x) * limiting_orientation.x;
    	}
    	if (fabs(orientation.y) > limiting_orientation.y) {
    		orientation.y = orientation.y/fabs(orientation.y) * limiting_orientation.y;
    	}
    	if (fabs(orientation.z) > limiting_orientation.z) {
    		orientation.z = orientation.z/fabs(orientation.z) * limiting_orientation.z;
    	}
    	return orientation;
    }
};

#endif