#include "ServoControl.h"


void update_servos(State* state)
{

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
}
