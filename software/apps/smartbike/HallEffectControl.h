#ifndef HALLEFFECTCONTROL_H_
#define HALLEFFECTCONTROL_H_

#include <stdbool.h>
#include "common.h"

/*****************************************************************************
                              Initialization
*****************************************************************************/

/* initialize the hall effect sensor */
void initializeHallEffect( );





/*****************************************************************************
                            Interrupt Handlers
*****************************************************************************/

/* We are getting digital readings based on threshold voltage values from    
 * hysteresis modules. Use these functions to emulate a quadrature decoder
 */
void HallEffectHandler_h1();
void HallEffectHandler_h2();

/*****************************************************************************
                               Retrieve Data
*****************************************************************************/

/* Using two hall effect sensors we can tell if we are back pedaling */
bool getPedalingDirection();

/* Effectively a quadrature-like count and time */
DataPair getCount();

#endif // HALLEFFECTCONTROL_H_
