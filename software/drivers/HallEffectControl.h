#ifndef HALLEFFECTCONTROL_H_
#define HALLEFFECTCONTROL_H_

#include <stdbool.h>
#include "common.h"
#include "BikeState.h"
#include "BikeTimers.h"

#define MM_PER_INT 1050// 997 is true value

/*****************************************************************************
                            Interrupt Handlers
*****************************************************************************/

//stores milliseconds
//sets wheel flag true
void wheel_interrupt_handler(State* bike);

//call this when flags[wheel_flag] is true
//updates target state and updates 
//deltas for use with braking logic
void update_target_state(State* bike);

// pedalling speed in radians
float get_pedalling_speed(const uint32_t *prev_count, const uint32_t *curr_count);

#endif // HALLEFFECTCONTROL_H_
