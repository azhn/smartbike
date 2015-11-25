#ifndef LIGHTACTION_H_
#define LIGHTACTION_H_

#include "BikeState.h"

/*typedef struct LightAction {
    LightType pos;
    LightState state;
} LightAction;*/

typedef enum LightAction {
    LIGHT_ACTION_NONE = 0,
    LIGHT_ACTION_LEFT_TURN = 1,
    LIGHT_ACTION_RIGHT_TURN =2,
     _BRAKE_ADDITION = 3,
    LIGHT_ACTION_BRAKE = 3,
    LIGHT_ACTION_LEFT_TURN_BRAKE = 4,
    LIGHT_ACTION_RIGHT_TURN_BRAKE = 5,
    _NUM_LIGHT_ACTION = 6
} LightAction;
 
// enum LedType {
//     LED_LEFT_INDICATOR = 0,
//     LED_RIGHT_INDICATOR,
//     LED_G1,
//     LED_G2,
//     LED_G3,
//     LED_G4,
//     LED_G5,
//     LED_G6,
//     LED_G7,
//     LED_G8,
//     _NUM_LEDS
// };

const static LightState _light_action_to_states[_NUM_LIGHT_ACTION][_NUM_LIGHT_TYPE] = {
    // CENTER                   LEFT                    RIGHT
    {LIGHT_STATE_DIM_ON,        LIGHT_STATE_OFF,        LIGHT_STATE_OFF},      // LIGHT_ACTION_NONE 
    {LIGHT_STATE_DIM_ON,        LIGHT_STATE_BLINKING,   LIGHT_STATE_OFF},      // LIGHT_ACTION_LEFT_TURN,
    {LIGHT_STATE_DIM_ON,        LIGHT_STATE_OFF,        LIGHT_STATE_BLINKING}, // LIGHT_ACTION_RIGHT_TURN,
    {LIGHT_STATE_ON,            LIGHT_STATE_ON,         LIGHT_STATE_ON},       // LIGHT_ACTION_BRAKE
    {LIGHT_STATE_ON,            LIGHT_STATE_BLINKING,   LIGHT_STATE_ON},       // LIGHT_ACTION_LEFT_TURN_BRAKE,
    {LIGHT_STATE_ON,            LIGHT_STATE_OFF,        LIGHT_STATE_BLINKING}  // LIGHT_ACTION_RIGHT_TURN_BRAKE,

};

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
