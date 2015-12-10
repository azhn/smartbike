#include "BikeTimers.h"
#include "ServoControl.h"
#include "led.h"

static const uint16_t REAR_GEAR_UP[6] = {335, 358, 364, 387, 400, 400};
static const uint16_t REAR_GEAR_DOWN[6] = {322, 340, 357, 376, 400, 400};
static const uint16_t FRONT_GEAR_UP[6] = {305, 305, 305, 285, 285, 225};
static const uint16_t FRONT_GEAR_DOWN[6] = {305, 305, 305, 305, 325, 230};


void update_servos(State* state)
{
    static uint32_t timer = 0;

    static bool over_shoot = false;

	if(state->target_gear >= NUM_GEARS)
	{
		state->target_gear = NUM_GEARS - 1;
	}

    if(state->target_gear == state->curr_gear)
    {
    } 
	else 
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
                pca9685_setPWM(REAR_PWM, 0, REAR_GEAR_UP[state->target_gear], PWM0_ADDR);
                pca9685_setPWM(FRONT_PWM, 0, FRONT_GEAR_UP[state->target_gear], PWM0_ADDR);
        }
        else
        {
                pca9685_setPWM(REAR_PWM, 0, REAR_GEAR_DOWN[state->target_gear], PWM0_ADDR);
                pca9685_setPWM(FRONT_PWM, 0, FRONT_GEAR_DOWN[state->target_gear], PWM0_ADDR);
        }

        state->curr_gear = state->target_gear;
    }
     state->flags[PEDAL_FLAG] = false;
}
