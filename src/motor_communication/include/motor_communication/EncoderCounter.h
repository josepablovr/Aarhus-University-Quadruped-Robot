#ifndef ENCODERCOUNTER_H
#define ENCODERCOUNTER_H

#include <ctime>
#include <cmath>

class EncoderCounter {
private:
    int counter;            // Turn Counter
    int previous_position;
    

    float multi_turn_angle;
    float displacement;
    float initial_angle;

public:
    EncoderCounter();
    void start(float initial_pos_multi, float initial_pos_single);
    float get_multi_turn(float current_position);
};

#endif
