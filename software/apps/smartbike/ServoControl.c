#include "ServoControl.h"


void update_servos(State* state)
{

	if(state.target_gear_state > state.curr_gear_state)
	{
	    state.shift_dir = UP; 
	}
  	else if(gear < state.curr_gear_state)
  	{
    	state.shift_dir = DOWN;
  	}
  
  	if(state.dir == UP) 
  	{
    	pca9685_setPWM(REAR_PWM, 0, REAR_GEAR_UP[gear]);
    	pca9685_setPWM(FRONT_PWM, 0, FRONT_GEAR_UP[gear]);
  	}
  	else
  	{
    	pca9685_setPWM(REAR_PWM, 0, REAR_GEAR_DOWN[gear]);
    	pca9685_setPWM(FRONT_PWM, 0, FRONT_GEAR_DOWN[gear]);
  	}
  	state.curr_gear_state = gear;
}
