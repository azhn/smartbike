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
	WHEEL_FLAG,
	PEDAL_FLAG,
	SHIFT_UP_FLAG,
	SHIFT_DOWN_FLAG,
	LEFT_TURN_FLAG,
	RIGHT_TURN_FLAG,
        MANUAL_MODE_SWITCH_FLAG,
        _NUM_FLAGS
};

// TODO: Double check these values. Some are placeholders.
static const uint8_t _pin_mappings[_NUM_FLAGS] = {
    6,  // WHEEL_FLAG,
    5,  // PEDAL_FLAG,
    4,  // SHIFT_UP_FLAG,
    3,  // SHIFT_DOWN_FLAG,
    9,  // LEFT_TURN_FLAG,
    10, // RIGHT_TURN_FLAG,
    8   // MANUAL_MODE_SWITCH_FLAG,
};



typedef struct State
{
    //speed stuff
    uint32_t last_milli;
    uint32_t curr_milli;
    uint32_t last_delta;
    uint32_t curr_delta;

    //shifting stuff
    bool manual_shifting;
    bool shift_dir;
    uint8_t target_gear;
    uint8_t curr_gear;

    // light stuff
    // Should be LIGHT_STATE_BLINKING_ON or LIGHT_STATE_BLINKING_OFF only
    LightState blinking_light_output; 

    //flags
    bool flags[_NUM_FLAGS];

    const uint8_t* pin_mappings;
}State;

bool test_milli_count_flag;

struct State* create_state();

void destroy_state(State* state);

#endif // BIKE_STATE_H
