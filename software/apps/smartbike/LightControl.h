#include <stdbool.h>
#include <stdint.h>

/***************************************************************************
                                  Types
***************************************************************************/
typedef enum {OFF, BLINKING, ON} LightState;

typedef enum {BRAKE=0, FRONT=1, BACK=2, 
              TURN=3, _LIGHT_TYPE_SIZE=4} LightType;

typedef enum {CENTER=0, LEFT=0, RIGHT=1, _LIGHT_POS_SIZE =2} LightPosition;

typedef struct {
    LightState _state;
    uint32_t _gpio_pin;
} Light;

/****************************************************************************
                                 Globals
*****************************************************************************/
/* index by light type, then position */
Light lights[_LIGHT_TYPE_SIZE][_LIGHT_POS_SIZE]; 

/*****************************************************************************
                              Initialization
*****************************************************************************/
/* initialize the lights */
void initializeLights();

/*****************************************************************************
                          Turning Lights On/Off
*****************************************************************************/
/* generic function to set any light state */
void setLightState(LightType type, LightPosition pos, LightState state );

/* generic function to get any light state */
LightState getLightState( LightType type, LightPosition pos);

/* turn on front light with the specified state */
void setFrontLight( LightState state );

/* set the a turn light with the specified state */
void setTurnLight(LightState state, LightPosition pos);

/* set the back light with the specified state */
void setBackLight(LightState state);

/* set the brake light with the specified state */
void setBrakeLight(LightState state);

/*****************************************************************************
                           Check Light Status
*****************************************************************************/
/* check the front light status */
LightState getfrontLight( );

/* check the turn light status */
LightState getTurnLight(LightPosition pos);

/* check the back light status */
LightState backLightStatus( );

/*****************************************************************************
                             Global Control
*****************************************************************************/

/* turn off all lights */
void turnOffAllLights( );

