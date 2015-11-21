#ifndef LIGHTCONTROL_H_
#define LIGHTCONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "pwm.h"
#define LED_LIGHTS_SIZE 10
/***************************************************************************
                                  Types
***************************************************************************/
typedef enum LightState {
    LIGHT_STATE_OFF=0,
    LIGHT_STATE_BLINKING=1,
    LIGHT_STATE_ON=2,
    _NUM_LIGHT_STATE=3
} LightState;

typedef enum LightType {
    BRAKE=0,
    LEFT_TURN=1,
    RIGHT_TURN=2,
    _LIGHT_TYPE_SIZE=3
} LightType;

typedef struct {
    pwm_address_t _address;
    LightState _state;
} Light;

/****************************************************************************
                                 Globals
*****************************************************************************/
/* index by light type, then position */
const static pwm_address_t _rear_light_address[_LIGHT_TYPE_SIZE] = {0/*INSERT ADDR*/,0/*INSERT ADDR*/,0/*INSERT ADDR*/};
const static pwm_address_t _led_light_address[LED_LIGHTS_SIZE] = {0/*INSERT ADDR*/,0/*INSERT ADDR*/,0/*INSERT ADDR*/,
                                                            0/*INSERT ADDR*/,0/*INSERT ADDR*/,0/*INSERT ADDR*/,
                                                            0/*INSERT ADDR*/,0/*INSERT ADDR*/,0/*INSERT ADDR*/,
                                                            0/*INSERT ADDR*/};
const static pwm_t _light_state_pwm[_NUM_LIGHT_STATE] = {0/*INSERT PWM*/,0/*INSERT PWM*/,0/*INSERT PWM*/};

Light _rear_lights[_LIGHT_TYPE_SIZE]; 
Light _led_lights[LED_LIGHTS_SIZE];

/*****************************************************************************
                              Initialization
*****************************************************************************/
/* initialize the lights */
void initializeLights();

/*****************************************************************************
                          Turning Lights On/Off
*****************************************************************************/
/* generic function to set any light state */
void setRearLightState(LightType type, LightState state );
void setLEDLightState(uint8_t type, LightState state );

/* generic function to get any light state */
LightState getRearLightState( LightType type );
LightState getLEDLightState( uint8_t pos );

/*****************************************************************************
                             Global Control
*****************************************************************************/

/* turn off all lights */
void turnOffAllLights( );

#endif // LIGHTCONTROL_H_
