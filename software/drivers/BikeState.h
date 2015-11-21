#ifndef BIKE_STATE_H
#define BIKE_STATE_H

#include <stdlib.h>
#include <stdbool.h>


#define UP true
#define DOWN false
#define NUM_FLAGS 6


enum FLAGS
{
	wheel_flag,
	pedal_flag,
	shift_up_flag,
	shift_down_flag,
	left_turn_flag,
	right_turn_flag
};




typedef struct State
{
    //speed stuff
    unsigned long last_milli;
    unsigned long curr_milli;
    unsigned long last_delta;
    unsigned long curr_delta;

    //shifting stuff
    bool shift_dir; 
    int target_gear;
    int curr_gear;

    //light stuff
    bool strobe_dir; 

	//flags
	bool flags[NUM_FLAGS];
}State;


struct State* create_state();

void destroy_state(State* state);

#endif
