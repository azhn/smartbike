#include "ServoControl.h"

static const uint16_t REAR_GEAR_UP[6] = {310, 340, 340, 352, 365, 372};
static const uint16_t REAR_GEAR_DOWN[6] = {308, 340, 315, 327, 340, 372};
static const uint16_t FRONT_GEAR_UP[6] = {400, 400, 313, 313, 313, 313};
static const uint16_t FRONT_GEAR_DOWN[6] = {400, 440, 313, 313, 313, 313};

void update_servos(State* state)
{
	if(state->target_gear == state->curr_gear)
	{
		return;
	}

	if(state->target_gear > state->curr_gear)
	{
	    state->shift_dir = UP; 
	}
  	else if(state->target_gear < state->curr_gear)
  	{
    	state->shift_dir = DOWN;
  	}
  
  	if(state->shift_dir == UP) 
  	{
    	pca9685_setPWM(REAR_PWM, 0, REAR_GEAR_UP[state->target_gear]);
    	pca9685_setPWM(FRONT_PWM, 0, FRONT_GEAR_UP[state->target_gear]);
  	}
  	else
  	{
    	pca9685_setPWM(REAR_PWM, 0, REAR_GEAR_DOWN[state->target_gear]);
    	pca9685_setPWM(FRONT_PWM, 0, FRONT_GEAR_DOWN[state->target_gear]);
  	}
  	state->curr_gear = state->target_gear;
	state->flags[pedal_flag] = false;
}
