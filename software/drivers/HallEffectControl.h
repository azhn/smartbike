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
void wheel_speed_handler();
void pedalling_speed_hander();

/*****************************************************************************
                               Retrieve Data
*****************************************************************************/

/* Effectively a quadrature-like count and time */
DataPair getCount();

#endif // HALLEFFECTCONTROL_H_
