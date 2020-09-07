#ifndef __TIMER__
#define __TIMER__

#include <arduino.h>

// Used to time the length of functions or create a delay.
// Can be set to check milliseconds or microseconds.
struct Timer {
    unsigned long time_start;
    unsigned long time_finish;
    unsigned long (*checkTime)();

    Timer(unsigned long ntime_finish=0) {
        time_finish = ntime_finish;
        checkTime = millis;
        reset();
    }

    void usePrecision()  {
        checkTime = micros;
        reset();
    }

    // Starts the timer to run for t ms.
    void reset(unsigned long t=0) {
        time_start = checkTime();
        if (t) {time_finish = t;} // If t is unset, reset with existing time. Else, reset with new time
    }

    // Returns whether the time has elapsed
    bool timeOut() {
        if (checkTime() >= time_start + time_finish) {
            return true;
        } else {
            return false;
        }
    }

    int dt() {
        return checkTime() - time_start;
    }
};

#endif