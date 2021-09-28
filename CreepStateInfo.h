#ifndef __CREEPSI__
#define __CREEPSI__

#include "Timer.h"


class CreepStateInfo {
    int id;
    float motion_period;
    float state_period;

    Timer state_periodr;
    bool is_in_startup;
    bool time_to_end;
    int next_state;

public:
    CreepStateInfo() = default;

    CreepStateInfo(int n_id, float nmotion_period, float nstate_period) {
        state_period = nstate_period;
        motion_period = nmotion_period;
        if (state_period != TIME_INFINITE) {
            state_periodr.reset(state_period * 1.05);
        }
        id = n_id;
        is_in_startup = true;
        time_to_end = false;
        next_state = id;
    }

    void reset() {
        is_in_startup = true;
        time_to_end = false;
        next_state = id;
    }

    void setStarted() { // "finishStartup?"
        is_in_startup = false;
        state_periodr.reset();
    }

    void setEnded() { // "endEarly"?
        time_to_end = true;
    }

    void setNextState(int n_state) {
        next_state = n_state;
    }

    float getMotionPeriod() {
        return motion_period;
    }

    int getNextState() {
        return next_state;
    }

    bool isInStartup() {
        return is_in_startup;
    }

    bool isTimeToEnd() {
        return (state_period != TIME_INFINITE && state_periodr.timeOut()) || time_to_end;
    }


};

#endif