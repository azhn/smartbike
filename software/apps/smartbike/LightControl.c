#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "pwm.h"
#include "LightControl.h"


void initializeLights() {
    int i;
    for(i=0; i<_LIGHT_TYPE_SIZE; ++i) {
        _rear_lights[i]._state = LIGHT_STATE_OFF;
        _rear_lights[i]._address = _rear_light_address[i];
        pca9685_setPWM(_rear_lights[i]._address, 0, _light_state_pwm[LIGHT_STATE_OFF]);
    }

    for(i=0; i<LED_LIGHTS_SIZE; ++i) {
        _led_lights[i]._state = LIGHT_STATE_OFF;
        _led_lights[i]._address = _led_light_address[i];
        pca9685_setPWM(_led_lights[i]._address, 0, _light_state_pwm[LIGHT_STATE_OFF]);
    }
}


void setRearLightState(LightType type, LightState state ) {
    _rear_lights[type]._state = state; 
    /* SET PWM */
    pca9685_setPWM(_rear_lights[type]._address, 0, _light_state_pwm[state]);
}

void setLEDLightState(uint8_t pos, LightState state ) {
    assert(pos < LED_LIGHTS_SIZE);
    _led_lights[pos]._state = state; 
    /* SET PWM */
    pca9685_setPWM(_led_lights[pos]._address, 0, _light_state_pwm[state]);
}

LightState getRearLightState( LightType type ) {
    return _rear_lights[type]._state; 
}

LightState getLEDLightState( uint8_t pos ) {
    assert(pos < LED_LIGHTS_SIZE);
    return _led_lights[pos]._state; 
}

void turnOffAllLights( ) {
    int i;
    for(i=0; i<_LIGHT_TYPE_SIZE; ++i) {
        _rear_lights[i]._state = LIGHT_STATE_OFF;
        pca9685_setPWM(_rear_lights[i]._address, 0, _light_state_pwm[LIGHT_STATE_OFF]);
    }
    for(i=0; i<LED_LIGHTS_SIZE; ++i) {
        _led_lights[i]._state = LIGHT_STATE_OFF;
        /* SET PWM */
        pca9685_setPWM(_led_lights[i]._address, 0, _light_state_pwm[LIGHT_STATE_OFF]);
    }
}


