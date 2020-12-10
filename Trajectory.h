#ifndef __TRAJECTORY__
#define __TRAJECTORY__
#include "Timer.h"

#define INFINITE_SPEED 10000

template <class State>
class TrajectoryInfo {
        bool is_active;
        Timer timer;

        float speed;
        State goal_state;

public:
        TrajectoryInfo() {
            is_active = false;
        }

        TrajectoryInfo(State beginning_state) {
            is_active = false;
            speed = INFINITE_SPEED;
            goal_state = beginning_state;
            timer.reset();
        }

        void setSpeed(float n_speed) {
            speed = n_speed;
        }

        void startGoal(State n_goal) {
            timer.reset();// thus thing...
            goal_state = n_goal;
            is_active = true;
        }

        void updateGoal(State n_goal) {
            goal_state = n_goal;
            is_active = true;
        }

        State getNextState(State current_state) {
            float dt = timer.dt();
            timer.reset();

            if (!is_active) {
                return current_state;
            }

            if (speed == INFINITE_SPEED) {
                return goal_state;
            } else {
                float maximum_distance_moved = speed * dt;
                float distance_to_goal = getDistanceToGoal(current_state);

                if (distance_to_goal > maximum_distance_moved) {
                    State direction_to_goal = getDirectionToGoal(current_state);
                    return current_state + direction_to_goal * maximum_distance_moved;
                } else {
                    end();
                    return goal_state;
                }
            }
        }

        void syncTimer() {
            timer.reset();
        }



private:
        float getDistanceToGoal(State current_state) {
            return (goal_state - current_state).norm();
        }

        State getDirectionToGoal(State current_state) {
            return (goal_state - current_state)/getDistanceToGoal(current_state);
        }

public:
        bool hasReachedGoal() {
            return !is_active;;
        }

        bool isInMotion() {
            return is_active;
        }

        void end() {
            is_active = false;
        }
    };

#endif