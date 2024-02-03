#include "EncoderCounter.h"
#include <cmath>

EncoderCounter::EncoderCounter() : counter(0), previous_position(0),multi_turn_angle(0), displacement(0), initial_angle(0) {}

void EncoderCounter::start(float initial_pos_multi, float initial_pos_single) {
	
    counter = initial_pos_multi / 40;
    previous_position = initial_pos_single;
    initial_angle = initial_pos_multi;
   
}

float EncoderCounter::get_multi_turn(float current_position) {
 
    // Displacement based on position
    float dPp = current_position - previous_position;
    
    
    if (std::abs(dPp) < 35) {
        // No abrupt changes means that it stayed in the same turn
        counter = counter; // Stays the same
    } else if (-dPp < 0) {
        // Clockwise movement
        counter += -1;
    } else {
        // Counterclockwise
        counter += 1;
    }

    multi_turn_angle = counter * 40 + current_position;

    
    previous_position = current_position;

    return multi_turn_angle;
}
