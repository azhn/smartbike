#include <assert.h>    
#include <stdbool.h>
#include <stdint.h>

#include "BikeState.h"
#include "LightControl.h"
#include "LightAction.h"

static void set_led_turn_indicator(const LightState* left_state, const LightState* right_state) {
    if (*left_state == LIGHT_STATE_BLINKING) {
        setLEDLightState(LED_LEFT_INDICATOR, LIGHT_STATE_ON);
        setLEDLightState(LED_RIGHT_INDICATOR, LIGHT_STATE_OFF);
        return;
    }
    
    if (*right_state == LIGHT_STATE_BLINKING) {
        setLEDLightState(LED_RIGHT_INDICATOR, LIGHT_STATE_ON);
        setLEDLightState(LED_LEFT_INDICATOR, LIGHT_STATE_OFF);
        return;
    }
    
    setLEDLightState(LED_LEFT_INDICATOR, left_state);
    setLEDLightState(LED_RIGHT_INDICATOR, right_state);
}

static void set_led_gear_indicator(uint8_t curr_gear) {
    assert (curr_gear <= _NUM_LEDS - LED_G1 + 1 && curr_gear >= 0);
    uint8_t i;
    // run through all leds, check state, ensure that correct lights are on
    for (i=LED_G1; i <= curr_gear + LED_G1; ++i) {
        if (getLEDLightState(i) != LIGHT_STATE_ON) {
            setLEDLightState(i, LIGHT_STATE_ON);
        } 
    } 
    for (i=curr_gear + LED_G1 + 1; i <= LED_G8; ++i) {
        if (getLEDLightState(i) != LIGHT_STATE_OFF) {
            setLEDLightState(i, LIGHT_STATE_OFF);
        }
    }
}

static void set_brake_indicator(const State* state, LightAction* light_action) {
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

static LightAction* light_action_to_states(const State* state, const LightAction* light_action) {
    assert (state != NULL && light_action != NULL);
    assert (*light_action < _NUM_LIGHT_ACTION);
    LightAction* ret;
    uint8_t i;
    for(i=CENTER_LIGHT; i < _NUM_LIGHT_TYPE; ++i) {
        if (_light_action_to_states[i] == LIGHT_STATE_BLINKING) {
            ret = state-> 
        } else {
            ret = _light_action_to_states[i];
        }
    } 
}

void performLightAction(const State* bike, const LightAction* light_action, uint8_t count) {
    uint8_t i;
    LightState left_state;
    LightState right_state;
    for(i=0; i<count; ++i) {
        // set light states
        if (light_action[i].state == LIGHT_STATE_BLINKING) {
            setRearLightState(light_action[i].pos, bike->blinking_light_output);
        } else {
            setRearLightState(light_action[i].pos, light_action[i].state);
        }
    }

    }
    // check back light turn signals
    // ensure only one is set to blinking
    left_state = getRearLightState(LEFT_TURN);
    right_state = getRearLightState(RIGHT_TURN);
    assert (left_state == LIGHT_STATE_BLINKING  && 
            right_state == LIGHT_STATE_BLINKING);

    // set turn signal front led
    set_led_turn_indicator(&left_state, &right_state);

    // check brake light and set appropriate back light

    // check current gear and set front LEDs
    set_led_gear_indicator(bike->curr_gear)
}
