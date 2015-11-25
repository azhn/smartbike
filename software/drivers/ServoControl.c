#include "BikeTimers.h"
#include "ServoControl.h"

static const uint16_t REAR_GEAR_UP[7] = {325, 345, 352, 352, 365, 372, 372};
static const uint16_t REAR_GEAR_DOWN[7] = {305, 318, 330, 330, 345, 372, 372};
static const uint16_t FRONT_GEAR_UP[7] = {430, 430, 430, 344, 344, 349, 349};
static const uint16_t FRONT_GEAR_DOWN[7] = {425, 425, 425, 344, 344, 349, 349};


/*static const uint16_t REAR_GEAR_UP[7] =           {325, 345, 352, 352, 365, 372, 372};
static const uint16_t REAR_GEAR_UP_OVERSHOOT[7] = {325, 345, 352, 352, 365, 372, 372};

static const uint16_t REAR_GEAR_DOWN[7] =           {308, 318, 330, 330, 345, 372, 372};
static const uint16_t REAR_GEAR_DOWN_OVERSHOOT[7] = {303, 318, 330, 330, 345, 372, 372};

static const uint16_t FRONT_GEAR_UP[7] =           {430, 430, 430, 366, 366, 366, 366};
static const uint16_t FRONT_GEAR_UP_OVERSHOOT[7] = {430, 430, 430, 310, 310, 310, 310};

static const uint16_t FRONT_GEAR_DOWN[7] =           {425, 425, 425, 366, 366, 366, 366};
static const uint16_t FRONT_GEAR_DOWN_OVERSHOOT[7] = {425, 425, 425, 366, 366, 366, 366};
*/
// REAR_UP:        325 345 352
// REAR_UP_OVER:   325 345 352
// REAR_DOWN:      308 318 330
// REAR_DOWN_OVER: 300 318 330

// FRONT_UP:       430 430 430
// FRONT_UP_OVER:  430 430 430
// FRONT_DOWN:     425 425 425
// FRONT_DOWN_OVER:425 425 425
// 
/*static const uint16_t REAR_GEAR_UP[7] =           {325, 345, 352,352,352,352,352};
static const uint16_t REAR_GEAR_UP_OVERSHOOT[7] = {325, 345, 330,352,352,352,352};

static const uint16_t REAR_GEAR_DOWN[7] =           {308, 318, 330,330,330,330,330};
static const uint16_t REAR_GEAR_DOWN_OVERSHOOT[7] = {300, 318, 330,330,330,330,330};

static const uint16_t FRONT_GEAR_UP[7] =           {425, 425, 425, 366,366,366,366};
static const uint16_t FRONT_GEAR_UP_OVERSHOOT[7] = {425, 425, 425, 302,302,302,302};

static const uint16_t FRONT_GEAR_DOWN[7] =           {425, 425, 425, 366,366,366,366};
static const uint16_t FRONT_GEAR_DOWN_OVERSHOOT[7] = {425, 425, 425, 366,366,366,366};*/

void update_servos(State* state)
{
    static uint32_t timer = 0;

    static bool over_shoot = false;

    if(state->target_gear == state->curr_gear)
//    if(state->target_gear == state->curr_gear && !over_shoot)
    {
        //	return;
    } else {

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
        state->flags[pedal_flag] = false;
    }
}
