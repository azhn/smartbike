#include "LightControl.h"
#include "BikeState.h"

struct State* create_state()
{
    int i = 0;
    State* state = malloc(sizeof(struct State));
    if(state == NULL)
    {   
        exit(666);
    }   

    state->last_milli = 0;
    state->curr_milli = 0;
    state->last_delta = 0;
    state->curr_delta = 0;

    state->manual_shifting = false;
    state->shift_dir = UP; 
    state->target_gear = 0;
    state->curr_gear = 0;
    state->blinking_light_output = LIGHT_STATE_BLINKING_OFF; 

    for(i = 0; i < _NUM_FLAGS; ++i)
    {
        state->flags[i] = false;
    }
    state->pin_mappings = _pin_mappings;
    return state;
}

void destroy_state(State* state)
{
    if(state != NULL)
    {   
        free(state);
    }   
}
