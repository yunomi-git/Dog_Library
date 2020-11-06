#ifndef __SNSAVG__
#define __SNSAVG__

template<typename dataType> 
class SensorHistory {
    #define MAX_HISTORY_SIZE 30
    dataType value;
    dataType value_history[MAX_HISTORY_SIZE];
    int history_size;
    int index = 0;
public:
    SensorHistory() {history_size = MAX_HISTORY_SIZE;}

    SensorHistory(int size) {
        if (size < MAX_HISTORY_SIZE) {
            history_size = MAX_HISTORY_SIZE;
        } else {
            history_size = size;
        }
    }

    void updateHistory(dataType new_value) {
        value = value + (new_value - value_history[index])/(history_size);
        value_history[index] = new_value;
        index = (index + 1)%history_size;
    }

    dataType getPrevHistoryValue(int di) {
        return value_history[(index - di + history_size) % history_size];
    }

    dataType getValue() {
        return value;
    }
};

#endif