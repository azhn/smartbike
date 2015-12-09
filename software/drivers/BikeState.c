#include "LightControl.h"
#include "BikeState.h"

struct State* create_state()
{
    // int i = 0;
    State* state = malloc(sizeof(struct State));
    if(state == NULL)
    {   
        exit(666);
    }   

    // state->last_milli = 0;
    // state->curr_milli = 0;
    // state->last_delta = 0;
    // state->curr_delta = 0;

    // state->manual_shifting = false;
    // state->shift_dir = UP; 
    // state->target_gear = 0;
    // state->curr_gear = 0;
    // state->blinking_light_output = LIGHT_STATE_BLINKING_OFF; 

    // state->handle_left_turn = false;
    // state->handle_right_turn = false;

    // for(i = 0; i < _NUM_FLAGS; ++i)
    // {
    //     state->flags[i] = false;
    // }
    state->pin_mappings = _pin_mappings;
    return state;
}

void reset_bike_state(State* state)
{
    int i = 0;
    state->last_milli = 0;
    state->curr_milli = 0;
    state->last_delta = 0;
    state->curr_delta = 0;

    // state->manual_shifting = false;
    if (nrf_drv_gpiote_in_is_set(_pin_mappings[MANUAL_MODE_SWITCH_FLAG])) {
        state->manual_shifting = true;
    } else {
        state->manual_shifting = false;
    }
    state->shift_dir = UP; 
    state->target_gear = 0;
    state->curr_gear = 0;
    state->blinking_light_output = LIGHT_STATE_BLINKING_OFF; 

    state->handle_left_turn = false;
    state->handle_right_turn = false;

    for(i = 0; i < _NUM_FLAGS; ++i)
    {
        state->flags[i] = false;
    }
    // state->pin_mappings = _pin_mappings;
}

void destroy_state(State* state)
{
    if(state != NULL)
    {   
        free(state);
    }   
}
