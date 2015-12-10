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
//    if(state->target_gear == state->curr_gear && !over_shoot)
    {
        // led_on(LED_1);
        //	return;
    } else {

        // led_off(LED_1);
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
 /*           if (!over_shoot) {
                pca9685_setPWM(REAR_PWM, 0, REAR_GEAR_UP_OVERSHOOT[state->target_gear]);
                pca9685_setPWM(FRONT_PWM, 0, FRONT_GEAR_UP_OVERSHOOT[state->target_gear]);

                over_shoot = true;
                timer = get_millis();
            } else if (get_millis() - timer > 500) {*/
                pca9685_setPWM(REAR_PWM, 0, REAR_GEAR_UP[state->target_gear], PWM0_ADDR);
                pca9685_setPWM(FRONT_PWM, 0, FRONT_GEAR_UP[state->target_gear], PWM0_ADDR);
/*                over_shoot = false;
            }*/
        }
        else
        {
/*            if (!over_shoot) {
                pca9685_setPWM(REAR_PWM, 0, REAR_GEAR_DOWN_OVERSHOOT[state->target_gear]);
                pca9685_setPWM(FRONT_PWM, 0, FRONT_GEAR_DOWN_OVERSHOOT[state->target_gear]);

                over_shoot = true;
                timer = get_millis();
            } else if (get_millis() - timer > 500) {*/
                pca9685_setPWM(REAR_PWM, 0, REAR_GEAR_DOWN[state->target_gear], PWM0_ADDR);
                pca9685_setPWM(FRONT_PWM, 0, FRONT_GEAR_DOWN[state->target_gear], PWM0_ADDR);
/*                over_shoot = false;
            }*/
        }

        state->curr_gear = state->target_gear;
        // state->flags[PEDAL_FLAG] = false;
    }
     state->flags[PEDAL_FLAG] = false;
}
