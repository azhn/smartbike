#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "LightControl.h"


void initializeLights() {
    int i;
    for(i=0; i<_LIGHT_TYPE_SIZE; ++i) {
        rear_lights[i]._state = LIGHT_STATE_OFF;
        rear_lights[i]._address = _rear_light_address[i];
    }

    for(i=0; i<LED_LIGHTS_SIZE; ++i) {
        led_lights[i]._state = LIGHT_STATE_OFF;
        led_lights[i]._address = _led_light_address[i];
    }
}


void setRearLightState(LightType type, LightState state ) {
    rear_lights[type]._state = state; 
}

void setLEDLightState(uint8_t pos, LightState state ) {
    assert(pos < LED_LIGHTS_SIZE);
    led_lights[pos]._state = state; 
}

LightState getRearLightState( LightType type ) {
    return rear_lights[type]._state; 
}

LightState getLEDLightState( uint8_t pos ) {
    assert(pos < LED_LIGHTS_SIZE);
    return led_lights[pos]._state; 
}

void turnOffAllLights( ) {
    int i;
    for(i=0; i<_LIGHT_TYPE_SIZE; ++i) {
        rear_lights[i]._state = LIGHT_STATE_OFF;
        /* SET PWM */
    }
    for(i=0; i<LED_LIGHTS_SIZE; ++i) {
        rear_lights[i]._state = LIGHT_STATE_OFF;
        /* SET PWM */
    }
}


