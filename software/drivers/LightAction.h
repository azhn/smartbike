#ifndef LIGHTACTION_H_
#define LIGHTACTION_H_

#include "BikeState.h"

typedef struct LightAction {
    LightType pos;
    LightState state;
} LightAction;

void performLightAction(const State* state, const LightAction* light_action, uint8_t count);
enum LedType {
    LED_LEFT_INDICATOR = 0,
    LED_RIGHT_INDICATOR,
    LED_G1,
    LED_G2,
    LED_G3,
    LED_G4,
    LED_G5,
    LED_G6,
    LED_G7,
    LED_G8,
    _NUM_LEDS
};

// Takes the state of the rear turn lights, and correctly drives the led indicators
static void set_led_turn(const LightState* left_state, const LightState* right_state);

// takes the current gear and outputs the correct gear state on the leds
static void set_led_gear_indicator(uint8_t curr_gear);

// takes the current bike state, and rear light states, and sets brake lights accordingly
static void set_brake_indicator(const State* state,
                                const LightState* left_state,
                                const LightState* right_state);
#endif // LIGHTACTION_H_
