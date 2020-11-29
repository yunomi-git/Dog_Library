#ifndef __BALANCE__
#define __BALANCE__

#include "Rot.h"
#include "Dog.h"
#include "Timer.h"

#define DEFAULT_D_GAIN 0.04
#define DEFAULT_I_GAIN 0.015

class BalanceHandler {
    float measurement_interval = 3.0/1000;
    Timer measurement_update_timer;

    IMU_orientation = ROT_ZERO;
    IMU_rot_velocity = ROT_ZERO;
    Rot error_integrator = ROT_ZERO;

    float I_gain;
    float D_gain;


    RobotDog *dog;

    BalanceHandler() = default;
    BalanceHandler(RobotDog *n_dog) {
        dog = n_dog;
        I_gain = DEFAULT_I_GAIN;
        D_gain = DEFAULT_D_GAIN;
    }

    void setIDGains(float n_I_gain, float n_D_gain) {
        I_gain = n_I_gain;
        D_gain = n_D_gain;
    }

    void operate() {
        if (measurement_update_timer.timeOut()) {
            Rot IMU_orientation = dog->getBodyIMUOrientation_fG2B();
            Rot IMU_rot_velocity = dog->getBodyIMURotVelocity_fG2B();
            error_integrator += IMU_orientation;
    
            measurement_update_timer.reset();
        }
    }

    Rot getNextKinematic  BalancingOrientation() {
        Rot desired_orientation = -(IMU_orientation - IMU_rot_velocity * D_gain + error_integrator * I_gain);
        //Point desired_position = Point(0, 0, dog.getStartingHeight());
        //dog.moveBodyToOrientation(desired_orientation, TIME_INSTANT);
        //dog.moveBodyToPositionFromCentroid(desired_position, Frame::GROUND, TIME_INSTANT);
    }
};

#endif