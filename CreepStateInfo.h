#ifndef __CREEP__
#define __CREEP__

#include "Timer.h"

enum ActionMode {STARTUP, NORMAL, END};

class CreepStateInfo {
    typedef void (*actionFunction)(ActionMode);
    actionFunction action;
    int id;
    float period;

    Timer state_end_timer;
    bool is_in_startup;
    bool time_to_end;
    int next_state;

    CreepStateInfo() = default;

    CreepStateInfo(int n_id, action_function new_action, float new_period) {
        period = new_period;
        if (period != TIME_INFINITE) {
            state_end_timer.reset(period * 1.1);
        }
        id = n_id;
        action = new_action;
        is_in_startup = true;
        time_to_end = false;
        next_state = id;
    }

    void setStarted() {
        is_in_startup = false;
        state_end_timer.reset();
    }

    void reset() {
        is_in_startup = true;
        time_to_end = false;
        next_state = id;
    }

    bool isTimeToEnd() {
        return (period != TIME_INFINITE && state_end_timer.timeOut()) || time_to_end;
    }

    void setNextState(int n_state) {
        next_state = n_state;
    }

    void setEnded() {
        time_to_end = true;
    }

    void getActionPeriod() {
        return period;
    }

    int getNextState() {
        return next_state;
    }

    void startupAction() {
        action(STARTUP);
    }

    void normalAction() {
        action(NORMAL);
    }

    void endAction() {
        action(END);
    }
};