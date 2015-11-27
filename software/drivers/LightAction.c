#include <assert.h>    
#include <stdbool.h>
#include <stdint.h>

#include "BikeState.h"
#include "LightControl.h"
#include "LightAction.h"

static void set_led_turn_indicator(const LightState* led_turn_states) {
    assert(sizeof(led_turn_states) == _NUM_TURN_INDICATORS);
    uint8_t i;
    for (i=0; i<_NUM_TURN_INDICATORS; ++i) {
        setLEDLightState(i, led_turn_states[i]);
    }
}

static void set_led_gear_indicator(uint8_t curr_gear) {
    assert (curr_gear <= _NUM_GEAR_INDICATORS - LED_G1 + 1 && curr_gear >= 0);
    uint8_t i;
    // run through all leds, check state, ensure that correct lights are on
    for (i=LED_G1; i <= curr_gear + LED_G1; ++i) {
        setLEDLightState(i, LIGHT_STATE_ON);
    } 
    for (i=curr_gear + LED_G1 + 1; i <= LED_G8; ++i) {
        setLEDLightState(i, LIGHT_STATE_OFF);
    }
}

static void check_brake_indicator(const State* state, LightAction* light_action) {
    // Check if previous velocity*0.95 is greater than current velocity
    assert(state != NULL && light_action != NULL);
    assert(*light_action < _NUM_LIGHT_ACTION);

    if ((float)(state->last_delta * 0.95f) >= (float)(state->curr_delta)) {
        if (*light_action < _BRAKE_ADDITION) {
            *light_action += _BRAKE_ADDITION;         
        }
    } else {
        if (*light_action >= _BRAKE_ADDITION) {
            *light_action -= _BRAKE_ADDITION;
        }
    }
}

LightState* light_action_to_rear_light_states(const State* state, const LightAction* light_action) {
    assert (state != NULL && light_action != NULL);
    assert (*light_action < _NUM_LIGHT_ACTION);
    LightState* ret = (LightState*)malloc(_NUM_LIGHT_TYPE; * sizeof(LightState));
    uint8_t i;
    for(i=CENTER_LIGHT; i < _NUM_LIGHT_TYPE; ++i) {
        if (_light_action_to_rear_states[*light_action][i] == LIGHT_STATE_BLINKING) {
            ret[i] = state->blinking_light_output;
        } else {
            ret[i] = _light_action_to_rear_states[*light_action][i];
        }
    } 
    return ret;
}

LightState* light_action_to_turn_led_states(const State* state, const LightAction* light_action) {
    assert (state != NULL && light_action != NULL);
    assert (*light_action < _NUM_LIGHT_ACTION);
    LightState* ret = (LightState*) malloc(_NUM_TURN_INDICATORS * sizeof(LightState));
    uint8_t i;
    for(i=LED_LEFT_INDICATOR; i < _NUM_TURN_INDICATORS; ++i) {
        if (_light_action_to_turn_states[*light_action][i] == LIGHT_STATE_BLINKING) {
            ret[i] = state->blinking_light_output;
        } else {
            ret[i] = _light_action_to_turn_states[*light_action][i];
        }
    } 
    return ret;
}

void performLightAction(const State* bike, LightAction light_action) {
    LightState* all_rear_states;
    LightState* all_turn_led_states;

    // Check braking
    check_brake_indicator(bike, &light_action); 

    // Set back lights
    all_rear_states = light_action_to_rear_light_states(state, &light_action);
    setAllRearLightStates(all_rear_states);
    free(all_rear_states); // Must free all_rear_states because dynamically allocated

    // set led turn signals
    all_turn_led_states = light_action_to_turn_led_states(state, &light_action);
    set_led_turn_indicator(&all_turn_led_states);
    free(all_turn_led_states); // Must free all_turn_led_states because dynamically allocated

    // set led gear indicators
    set_led_gear_indicator(bike->curr_gear)
}
