#ifndef __RSERVODRIVER__
#define __RSERVODRIVER__

// #if (ARDUINO >= 100)
// #include "Arduino.h"
// #else
// #include "WProgram.h"
// #endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class RServoDriver : public Adafruit_PWMServoDriver {
#define DEF_ANGLE_MAX  (90.0)
#define DEF_PWM 		400
#define DEF_ANGLE_MIN (-90.0)
#define DEF_SIG_MIN    1450
#define DEF_SIG_MAX	   3460
#define DEF_SIG_MID	   2430

#define MAX_SERVO_NUM	16
#define TABLE_MID_INDEX 9

protected:
	struct servo_info {
		float sig_min;
		float sig_max;
		float angle_max;
		float angle_min;
		float angle_offset; // Horn starting angle AKA zero signal angle relative to desired frame origin in deg CCW (RHR).  
							// zs is (2455 is no table, table[9] if there is table)
		int direction;

		int *signals_table; // Table of angles. size should be 19. angle ordered as [-90, -80, -70....0...80, 90]
							  // NOTE: table signals zeroed at 2440. Default zeroed at 2455

		servo_info() {
			angle_max = DEF_ANGLE_MAX;
		 	angle_min = DEF_ANGLE_MIN;
		 	sig_max = DEF_SIG_MAX;
		 	sig_min = DEF_SIG_MIN;

		 	direction = 1;

		 	angle_offset = 0;
		 	signals_table = NULL;
		}

		// Converters
		int angleToPwm(float angle) {
			if (signals_table) { // input angle between -90, 90
				// First bring angle to correct frame
				angle = direction * (angle - angle_offset);
				// Limit the angle
				if (angle > 90)  {Serial.print(angle); angle = 90; Serial.println(" cut to 90");}
				if (angle < -90) {Serial.print(angle); angle = -90; Serial.println(" cut to -90");}
				// extract the base index. ex -87 -> -8 -> 1 (index)
				int idx_base = (int) (angle/10) + TABLE_MID_INDEX;
				int idx_interp;
				// obtain the index to interpolate to
				if (angle > 0) 	idx_interp = idx_base + 1; 
				else 			idx_interp = idx_base - 1;
				// corresponding signals to indices
				int sig_base = signals_table[idx_base];
				int sig_interp = signals_table[idx_interp];
				// amount to interpolate from base to interp
				float interp_ratio = (angle - 10 * (int) (angle/10)) / 10.0;
				// get the signal
				int signal = sig_base + abs(sig_interp - sig_base) * interp_ratio;
				return signal;
			} else {
				return round(sig_min + (angle - angle_min) * (sig_max - sig_min) / (angle_max - angle_min));
			}
		}

		float pwmToAngle(int pwm) {
			return angle_min + 1.0 * (pwm - sig_min) * (angle_max - angle_min) / (sig_max - sig_min);
		}

		// sets up the angle table
		void setTable(int *new_signals_table) {
			signals_table = new_signals_table;
		}

		// sets up the min/max angles to desired values
		void setAngles(float nangle_min, float nangle_max, int nsig_min, int nsig_max) {
			angle_max = nangle_max;
		 	angle_min = nangle_min;
		 	sig_max = nsig_max;
		 	sig_min = nsig_min;
		}

		// Reverses the direction of the servo.
		// Returns true: CCW is + AKA normal | false: CW is -
		bool reverseDirection() {
			float temp = sig_min;
			sig_min = sig_max;
			sig_max = temp;
			direction *= -1;
			return (sig_max > sig_min);
		}

		void setOffset(float offset) {
			angle_max -= angle_offset;
			angle_min -= angle_offset;
			angle_offset = offset;
			angle_max += angle_offset;
			angle_min += angle_offset;
		}
	};

	servo_info servo_list[MAX_SERVO_NUM];

public:
	RServoDriver() : Adafruit_PWMServoDriver() {
	}

	void defaultStartup() {
		begin();
  		setPWMFreq(DEF_PWM);  // set from 40 to 1600

	    // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
	    // some i2c devices dont like this so much so if you're sharing the bus, watch
	    // out for this!
	    Wire.setClock(400000);
	}

	// Sets the desired step count to that which most nearly corresponds with the angle
	void gotoAngle(int channel, float angle) {
		setPWM(channel, 0, servo_list[channel].angleToPwm(angle));
	}

	void reverseDirection(int channel) {
		servo_list[channel].reverseDirection();
	}

	void setOffset(int channel, float offset) {
		servo_list[channel].setOffset(offset);
	}

	servo_info * getServo(int channel) {
		return &(servo_list[channel]);
	}
};

#endif