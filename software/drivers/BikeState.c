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
    state->shift_dir = UP; 
    state->target_gear = 0;
    state->curr_gear = 0;
    state->strobe_dir = UP; 

	for(i = 0; i < NUM_FLAGS; ++i)
	{
		state->flags[i] = false;
	}

    return state;
}

void destroy_state(State* state)
{
    if(state != NULL)
    {   
        free(state);
    }   
}

