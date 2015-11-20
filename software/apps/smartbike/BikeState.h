#ifndef BIKE_STATE_H
#define BIKE_STATE_H

#include <stdlib.h>
#include <stdbool.h>


#define UP true
#define DOWN false

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
}State;


struct State* create_state();

void destroy_state(State* state);

#endif
