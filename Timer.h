#ifndef __TIMER__
#define __TIMER__

#include <arduino.h>

#define TIME_INFINITE -1

// TODO: cant do Timer a; a = Timer(xxx); or an error will happen...not sure why
// Used to time the length of functions or create a delay.
// Can be set to check milliseconds or microseconds.
// Interaction is always in seconds
struct Timer {
    unsigned long time_start;
    unsigned long time_finish;
    unsigned long (*checkTime)();
    bool using_precision;

    Timer(unsigned long ntime_finish=0) {
        checkTime = millis;
        using_precision = false;
        reset(ntime_finish);   
    }

    void usePrecision()  {
        checkTime = micros;
        using_precision = true;
        reset();
    }

    // Starts the timer to run for t ms.
    void reset(float t=0) {
        time_start = checkTime();
        if (t > 0) { // If t is unset, reset with existing time. Else, reset with new time
            if (using_precision) {
                time_finish = (unsigned long) (t * 1000000);
            } else {
                time_finish = (unsigned long) (t * 1000);
            }
        }
    }

    // Returns whether the time has elapsed
    bool timeOut() {
        if (checkTime() >= time_start + time_finish) {
            return true;
        } else {
            return false;
        }
    }

    float dt() {
        if (using_precision) {
            return (checkTime() - time_start) / 1000000.0;
        } else {
            return (checkTime() - time_start) / 1000.0;
        }
    }
};

#endif