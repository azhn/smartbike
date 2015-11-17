#ifndef LIGHTCONTROL_H_
#define LIGHTCONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#define LED_LIGHTS_SIZE 10
/***************************************************************************
                                  Types
***************************************************************************/
typedef enum {OFF=0, BLINKING=1, ON=2} LightState;

typedef enum {BRAKE=0, LEFT_TURN=2, RIGHT_TURN=3, _LIGHT_TYPE_SIZE=4} LightType;
typedef uint32_t pwm_address_t;

typedef struct {
    LightState _state;
    pwm_address_t _address;
} Light;

/****************************************************************************
                                 Globals
*****************************************************************************/
/* index by light type, then position */
Light rear_lights[_LIGHT_TYPE_SIZE]; 
Light led_lights[LED_LIGHTS_SIZE];

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
void setLEDLightState(uint8_t pos);

/* generic function to get any light state */
LightState getRearLightState( LightType type );
LightState getLEDLightState( uint8_t pos );

/*****************************************************************************
                             Global Control
*****************************************************************************/

/* turn off all lights */
void turnOffAllLights( );

#endif // LIGHTCONTROL_H_
