#ifndef LIGHTACTION_H_
#define LIGHTACTION_H_

#include "BikeState.h"

typedef struct LightAction {
    LightType pos;
    LightState state;
} LightAction

typedef enum LightAction {
    LIGHT_ACTION_NONE = 0,
    LIGHT_ACTION_BRAKE,
    LIGHT_ACTION_LEFT_TURN,
    LIGHT_ACTION_LEFT_TURN_BRAKE,
    LIGHT_ACTION_RIGHT_TURN,
    LIGHT_ACTION_RIGHT_TURN_BRAKE,
    _NUM_LIGHT_ACTION 
} LightAction

    LIGHT_STATE_OFF=0,
    LIGHT_STATE_ON,
    LIGHT_STATE_DIM_ON,
    LIGHT_STATE_BLINKING,
    LIGHT_STATE_BLINKING_OFF,
    LIGHT_STATE_BLINKING_ON,
    _NUM_LIGHT_STATE

static LightState[_NUM_LIGHT_ACTION][_NUM_LIGHT_TYPE] = {
{
    // CENTER                   LEFT                    RIGHT
    {LIGHT_STATE_ON_DIM,        LIGHT_STATE_OFF,        LIGHT_STATE_OFF}, // LIGHT_ACTION_NONE 
}
void performLightAction(const State* state, const LightAction* light_action, uint8_t count);
// Takes the state of the rear turn lights, and correctly drives the led indicators
static void set_led_turn(const LightState* left_state, const LightState* right_state);

// takes the current gear and outputs the correct gear state on the leds
static void set_led_gear_indicator(uint8_t curr_gear);

// takes the current bike state, and rear light states, and sets brake lights accordingly
static void set_brake_indicator(const State* state,
                                const LightState* left_state,
                                const LightState* right_state);
#endif // LIGHTACTION_H_
