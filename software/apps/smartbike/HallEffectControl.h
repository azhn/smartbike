#ifndef HALLEFFECTCONTROL_H_
#define HALLEFFECTCONTROL_H_

#include <stdbool.h>
#include "common.h"

#define MM_PER_INT 997

/*****************************************************************************
                            Interrupt Handlers
*****************************************************************************/

/* We are getting digital readings based on threshold voltage values from    
 * hysteresis modules. Use these functions to emulate a quadrature decoder
 */
void wheel_speed_handler(uint32_t* out);
void pedalling_speed_hander(uint32_t* out);

// bike speed in mm/ms = m/s. We can try to design to scale to prefix_m/s to increase granualarity
uint32_t get_bike_speed(const uint32_t *prev_count, const uint32_t *curr_count);

// pedalling speed in radians
float get_pedalling_speed(const uint32_t *prev_count, const uint32_t *curr_count);

#endif // HALLEFFECTCONTROL_H_
