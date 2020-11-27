#ifndef __CREEPSI__
#define __CREEPSI__

#include "Timer.h"


class CreepStateInfo {
   // actionFunction action;
    int id;
    float period;

    Timer state_end_timer;
    bool is_in_startup;
    bool time_to_end;
    int next_state;

public:
    CreepStateInfo() = default;

    CreepStateInfo(int n_id, float new_period) {
        period = new_period;
        if (period != TIME_INFINITE) {
            state_end_timer.reset(period * 1.05);
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

    void setStarted() {
        is_in_startup = false;
        state_end_timer.reset();
    }

    void setEnded() {
        time_to_end = true;
    }

    void setNextState(int n_state) {
        next_state = n_state;
    }

    float getActionPeriod() {
        return period;
    }

    int getNextState() {
        return next_state;
    }

    bool isInStartup() {
        return is_in_startup;
    }

    bool isTimeToEnd() {
        return (period != TIME_INFINITE && state_end_timer.timeOut()) || time_to_end;
    }


};

#endif