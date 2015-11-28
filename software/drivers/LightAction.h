#ifndef LIGHTACTION_H_
#define LIGHTACTION_H_

#include "BikeState.h"

/****************************************************************************
                               Data Types
****************************************************************************/

typedef enum LightAction {
    // Non-braking Light Actions
    LIGHT_ACTION_NONE = 0,
    LIGHT_ACTION_LEFT_TURN = 1,
    LIGHT_ACTION_RIGHT_TURN =2,
     _BRAKE_ADDITION = 3,

    // Braking Light actions
    LIGHT_ACTION_BRAKE = 3,
    LIGHT_ACTION_LEFT_TURN_BRAKE = 4,
    LIGHT_ACTION_RIGHT_TURN_BRAKE = 5,
    _NUM_LIGHT_ACTION = 6
} LightAction;
 
const static LightState _light_action_to_rear_states[_NUM_LIGHT_ACTION][_NUM_LIGHT_TYPE] = {
//  {CENTER                     LEFT                    RIGHT}
    {LIGHT_STATE_DIM_ON,        LIGHT_STATE_OFF,        LIGHT_STATE_OFF},      // LIGHT_ACTION_NONE 
    {LIGHT_STATE_DIM_ON,        LIGHT_STATE_BLINKING,   LIGHT_STATE_OFF},      // LIGHT_ACTION_LEFT_TURN,
    {LIGHT_STATE_DIM_ON,        LIGHT_STATE_OFF,        LIGHT_STATE_BLINKING}, // LIGHT_ACTION_RIGHT_TURN,
    {LIGHT_STATE_ON,            LIGHT_STATE_ON,         LIGHT_STATE_ON},       // LIGHT_ACTION_BRAKE
    {LIGHT_STATE_ON,            LIGHT_STATE_BLINKING,   LIGHT_STATE_ON},       // LIGHT_ACTION_LEFT_TURN_BRAKE,
    {LIGHT_STATE_ON,            LIGHT_STATE_OFF,        LIGHT_STATE_BLINKING}  // LIGHT_ACTION_RIGHT_TURN_BRAKE,

};

const static LightState _light_action_to_turn_states[_NUM_LIGHT_ACTION][_NUM_TURN_INDICATORS] = {
//  {RIGHT                      LEFT}
    {LIGHT_STATE_OFF,           LIGHT_STATE_OFF},           // LIGHT_ACTION_NONE 
    {LIGHT_STATE_OFF,           LIGHT_STATE_ON},            // LIGHT_ACTION_LEFT_TURN
    {LIGHT_STATE_ON,            LIGHT_STATE_OFF},           // LIGHT_ACTION_RIGHT_TURN
    {LIGHT_STATE_OFF,           LIGHT_STATE_OFF},           // LIGHT_ACTION_BRAKE
    {LIGHT_STATE_OFF,           LIGHT_STATE_ON},            // LIGHT_ACTION_LEFT_TURN_BRAKE
    {LIGHT_STATE_ON,            LIGHT_STATE_OFF}            // LIGHT_ACTION_RIGHT_TURN_BRAKE
};

/****************************************************************************
                               Utility Functions
****************************************************************************/

// Return LightState array that is dynamically allocated using malloc. Must be freed after use
LightState* light_action_to_rear_light_states(const State* state, const LightAction* light_action);

// Return LightState array that is dynamically allocated using malloc. Must be freed after use
LightState* light_action_to_turn_led_states(const State* state, const LightAction* light_action);


/****************************************************************************
                             Static Functions                               
****************************************************************************/

// takes the turn indicator states and outputs to leds
static void set_led_turn_indicators(const LightState* led_turn_states);

// takes the current gear and outputs the correct gear state on the leds
static void set_led_gear_indicator(uint8_t curr_gear);

// takes the current bike state, and rear light action, and sets brake lights accordingly
static void check_brake_indicator(const State* state, LightAction* light_action);


/****************************************************************************
                               Utility Functions
****************************************************************************/
void performLightAction(const State* bike, LightAction light_action);


#endif // LIGHTACTION_H_
