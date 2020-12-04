#ifndef __TRAJECTORY__
#define __TRAJECTORY__
#include "Timer.h"

template <class State>
class TrajectoryInfo {
        bool is_active;

        Timer timer;
        float prev_time;
        
        float final_time;
        State final_state;

      //  State state_adjustment;

public:
        TrajectoryInfo() {
            is_active = false;
        }

        void begin(State nfinal_state, float nfinal_time) {
            is_active = true;
            timer.reset();
            prev_time = 0;
            final_time = nfinal_time;
            final_state = nfinal_state;
            //state_adjustment = State{};
        }

        // void setStateAdjustment(State n_state_adjustment) {
        //   //  if (is_active) 
        //         state_adjustment = n_state_adjustment;
        // }

        State getNextState(State current_state) {
            if (!isActive()) {
                return current_state;
            } else {
                State next_state;
                float current_time = getCurrentTime();
                //current_state = current_state - state_adjustment;

                if (current_time < final_time) {
                    float scaling_factor = (current_time - prev_time) / (final_time - prev_time); 
                    next_state = current_state  + (final_state - current_state) * scaling_factor;
                    prev_time = current_time;
                } else {
                    next_state = final_state;
                    end();
                }

                next_state = next_state;// + state_adjustment;
                return next_state;
            }
        }

        void end() {
            final_time = 0;
            is_active = false;
        }

        bool isActive() {
            return is_active;
        }

        void adjustFinalTime(float n_final_time) {
            if (is_active)
                final_time = n_final_time;
        }

        void adjustFinalState(State n_final_state) {
            if (is_active)
                final_state = n_final_state;
        }

        State getRemainingState(State current_state) {
            return final_state - current_state;
        }

        float getRemainingTime() {
            return final_time - timer.dt();
        }

        // returns time in seconds
        float getCurrentTime() {
            if (is_active)
                return timer.dt();
            else
                return 0;
        }
    };

#endif