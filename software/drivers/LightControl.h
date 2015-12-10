#ifndef LIGHTCONTROL_H_
#define LIGHTCONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "pwm.h"

/******************************************************************************
                                 Defines
 *****************************************************************************/

#define REAR_LIGHT_PWM_ADDR PWM0_ADDR
#define LED_LIGHT_PWM_ADDR  PWM1_ADDR

/***************************************************************************
                                  Types
***************************************************************************/
typedef enum LightState {
    LIGHT_STATE_OFF=0,
    LIGHT_STATE_ON,
    LIGHT_STATE_DIM_ON,
    LIGHT_STATE_BLINKING,
    LIGHT_STATE_BLINKING_OFF,
    LIGHT_STATE_BLINKING_ON,
    _NUM_LIGHT_STATE
} LightState;

typedef enum LightType {
    CENTER_LIGHT=0, // RUNNING
    LEFT_TURN,
    RIGHT_TURN,
    _NUM_LIGHT_TYPE
} LightType;

enum LedType {
    // TURN INDICATOR LEDS
    LED_RIGHT_INDICATOR   = 0,
    LED_LEFT_INDICATOR    = 1,
    _NUM_TURN_INDICATORS  = 2,

    // GEAR INDICATOR LEDS
    LED_G1                = 2,
    LED_G2                = 3,
    LED_G3                = 4,
    LED_G4                = 5,
    LED_G5                = 6,
    LED_G6                = 7,
    _NUM_GEAR_INDICATORS  = 8,

    // SHIFTING MODE INFO LEDS
    LED_AUTOMATIC_INFO    = 8,
    LED_MANUAL_INFO       = 9,
    _NUM_INFO_INDICATORS  = 10,

    _NUM_LEDS             = 10
} LedType;

typedef struct Light {
    pwm_address_t _address;
    LightState _state;
} Light;


/****************************************************************************
                                 Globals
*****************************************************************************/
/* index by light type, then position */
const static pwm_address_t _rear_light_address[_NUM_LIGHT_TYPE] = {2,3,4};

// TODO: CHANGE THESE VALUES TO SOMETHING USEFUL
const static pwm_t _light_state_pwm[_NUM_LIGHT_STATE] = {
    0,      //LIGHT_STATE_OFF=0,
    4095,   //LIGHT_STATE_ON,
    2048,   //LIGHT_STATE_DIM_ON,
    0,   //LIGHT_STATE_BLINKING,
    0,      //LIGHT_STATE_BLINKING_OFF,
    4095    //LIGHT_STATE_BLINKING_ON,
};

Light _rear_lights[_NUM_LIGHT_TYPE]; 
Light _led_lights[_NUM_LEDS];

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
void setLEDLightState(uint8_t pos, LightState state );
void setAllRearLightStates(const LightState* states);

/* generic function to get any light state */
LightState getRearLightState( LightType type );
LightState getLEDLightState( uint8_t pos );

/*****************************************************************************
                             Global Control
*****************************************************************************/

/* turn off all lights */
void turnOffAllLights( );

#endif // LIGHTCONTROL_H_
