#ifndef __TIMER__
#define __TIMER__

#include <arduino.h>

// Used to time the length of functions or create a delay.
struct Timer {
    unsigned int time_start;
    unsigned int time_finish;

    // Starts the timer to run for t ms.
    void reset(unsigned int t=0) {
        time_start = millis();
        if (t) {time_finish = t;}
    }

    // Returns whether the time has elapsed
    bool timeOut() {
        if (millis() - time_start >= time_finish) {
            return true;
        } else {
            return false;
        }
    }

    int dt() {
        return millis() - time_start;
    }
};

#endif