//#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "pwm.h"
#include "LightControl.h"


void initializeLights() {
    uint8_t i;

    for (i=0; i<_NUM_LIGHT_TYPE; ++i) {
        _rear_lights[i]._address = _rear_light_address[i];
    }

    for (i=0; i<_NUM_LEDS; ++i) {
        _led_lights[i]._address = i;
    }
    turnOffAllLights();
}

void setAllRearLightStates(const LightState* state) {
    //// assert(sizeof(state)/sizeof(state[0]) == _NUM_LIGHT_TYPE);
    uint8_t i;
    for (i=CENTER_LIGHT; i < _NUM_LIGHT_TYPE; ++i) {
        setRearLightState((LightType)i, state[i]);
    }
}

void setRearLightState(LightType type, LightState state ) {
    //assert (type < _NUM_LIGHT_TYPE && state < _NUM_LIGHT_STATE);
    /* SET PWM */
    if (state == LIGHT_STATE_BLINKING) {
        state = LIGHT_STATE_BLINKING_ON;
    }
    if (state == _rear_lights[type]._state) {
        return;
    }
    
    _rear_lights[type]._state = state; 

    // pca9685_setPWM(_rear_lights[type]._address, 0, _light_state_pwm[state], REAR_LIGHT_PWM_ADDR);
    pca9685_setPin(_rear_lights[type]._address, _light_state_pwm[state], 0, REAR_LIGHT_PWM_ADDR);
}

void setLEDLightState(uint8_t pos, LightState state ) {
   // assert(pos < _NUM_LEDS);
    if (state == LIGHT_STATE_BLINKING) {
        state = LIGHT_STATE_BLINKING_ON;
    }
    if (state == _led_lights[pos]._state) {
        return;
    }
    _led_lights[pos]._state = state; 
    /* SET PWM */
    //pca9685_setPWM(_led_lights[pos]._address, 0, _light_state_pwm[state], LED_LIGHT_PWM_ADDR);
    pca9685_setPin(_led_lights[pos]._address, _light_state_pwm[state], 0, LED_LIGHT_PWM_ADDR);
}

LightState getRearLightState( LightType type ) {
    //assert (type < _NUM_LIGHT_TYPE);
    LightState ret;
    ret = _rear_lights[type]._state;
    if (ret == LIGHT_STATE_BLINKING_ON || ret == LIGHT_STATE_BLINKING_OFF) {
        ret = LIGHT_STATE_BLINKING;
    }
    return ret; 
}

LightState getLEDLightState( uint8_t pos ) {
   // assert(pos < _NUM_LEDS);
    LightState ret;
    ret = _led_lights[pos]._state;
    if (ret == LIGHT_STATE_BLINKING_ON || ret == LIGHT_STATE_BLINKING_OFF) {
        ret = LIGHT_STATE_BLINKING;
    }
    return ret; 
}

void turnOffAllLights( ) {
    int i;
    for(i=0; i<_NUM_LIGHT_TYPE; ++i) {
        _rear_lights[i]._state = LIGHT_STATE_OFF;
        // pca9685_setPWM(_rear_lights[i]._address, 0, _light_state_pwm[LIGHT_STATE_OFF], REAR_LIGHT_PWM_ADDR);
        pca9685_setPin(_rear_lights[i]._address, _light_state_pwm[LIGHT_STATE_OFF], 0, REAR_LIGHT_PWM_ADDR);
    }
    for(i=0; i<_NUM_LEDS; ++i) {
        _led_lights[i]._state = LIGHT_STATE_OFF;
        /* SET PWM */
        // pca9685_setPWM(_led_lights[i]._address, 0, _light_state_pwm[LIGHT_STATE_OFF], LED_LIGHT_PWM_ADDR);
        pca9685_setPin(_led_lights[i]._address, _light_state_pwm[LIGHT_STATE_OFF], 0, LED_LIGHT_PWM_ADDR);
    }
}
