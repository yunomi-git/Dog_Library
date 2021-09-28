#ifndef __RSERVO__
#define __RSERVO__

// #if (ARDUINO >= 100)
// #include "Arduino.h"
// #else
// #include "WProgram.h"
// #endif

#include <Servo.h>

class RServo : public Servo {
protected:
	// Servo servo;
	
	float angle_max;
	float angle_min;
	int pwm_min;
	int pwm_max;

	float angle_offset; // Horn starting angle in deg CCW (RHR)
	bool reverse_direction;	// normal: CCW is + | reverse: CW is +

	float angle_on;
	float angle_off;
public:
	RServo() {}

	RServo(float nangle_min, float nangle_max, int npwm_min, int npwm_max, float nspeed = 60) {
	 	angle_max = nangle_max;
	 	angle_min = nangle_min;
	 	pwm_max = npwm_max;
	 	pwm_min = npwm_min;

	 	reverse_direction = false;

	 	angle_on = 1500;
	 	angle_off = 1500;

	 	angle_offset = 0;
	}

	// void attach(int pin) {
	// 	servo.attach(pin);
 //        pinMode(pin, OUTPUT);
	// }

	// Reverses the direction of the servo.
	// Returns true: CCW is + AKA normal | false: CW is +
	bool reverseDirection() {
		float temp = pwm_min;
		pwm_min = pwm_max;
		pwm_max = temp;
		return (pwm_max > pwm_min);
	}

	void setOffset(float offset) {
		angle_max -= angle_offset;
		angle_min -= angle_offset;
		angle_offset = offset;
		angle_max += angle_offset;
		angle_min += angle_offset;
	}

	// Sets the desired step count to that which most nearly corresponds with the angle
	bool gotoAngle(float angle) {
		if (checkAngle(angle)) {
			writeMicroseconds(angleToPwm(angle));
			return true;
		} else {
			return false;
		}
	}

	bool checkAngle(float angle) {
		return ((angle >= angle_min) && (angle <= angle_max));
	}

	// Returns the current angular position corresponding to the step position in deg
	float getAngle() {
		return pwmToAngle(readMicroseconds());
	}

	// Sets on and off state for "boolean" servo
	void setStates(float ang_on, float ang_off) {
		angle_on = ang_on;
		angle_off = ang_off;
	}

	void gotoON() {
		gotoAngle(angle_on);
	}

	void gotoOFF() {
		gotoAngle(angle_off);
	}
private:
	// Converters
	int angleToPwm(float angle) {
		return round(pwm_min + (angle - angle_min) * (pwm_max - pwm_min) / (angle_max - angle_min));
	}

	float pwmToAngle(int pwm) {
		return angle_min + 1.0 * (pwm - pwm_min) * (angle_max - angle_min) / (pwm_max - pwm_min);
	}

	// Useful for maintaining speed
	void operate() {
		//servo.writeMicroseconds(current_pwm);
	}
};

#endif