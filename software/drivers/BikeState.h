#ifndef BIKE_STATE_H
#define BIKE_STATE_H

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "LightControl.h"

#define UP true
#define DOWN false


enum FLAGS
{
	wheel_flag,
	pedal_flag,
	shift_up_flag,
	shift_down_flag,
	left_turn_flag,
	right_turn_flag,
        _NUM_FLAGS
};




typedef struct State
{
    //speed stuff
    uint32_t last_milli;
    uint32_t curr_milli;
    uint32_t last_delta;
    uint32_t curr_delta;

    //shifting stuff
    bool shift_dir;
    uint8_t target_gear;
    uint8_t curr_gear;

    // light stuff
    // Should be LIGHT_STATE_BLINKING_ON or LIGHT_STATE_BLINKING_OFF only
    LightState blinking_light_output; 

    //flags
    bool flags[_NUM_FLAGS];
}State;

bool test_milli_count_flag;

struct State* create_state();

void destroy_state(State* state);

#endif // BIKE_STATE_H
