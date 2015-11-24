#ifndef BIKE_TIMERS_H
#define BIKE_TIMERS_H

#include <stdint.h>

#include "nrf.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "nrf51_bitfields.h"
#include "nrf_drv_config.h"

/******************************************************************************
 *  DEFINES
 *****************************************************************************/

#define BIKE_TIMER_PRESCALER        32  // Value of RTC1 PRESCALER Register
#define BIKE_TIMER_MAX_TIMERS       4   // Maximum number of simultaneous timers
#define BIKE_TIMER_OP_QUEUE_SIZE    4   // Size of timer operation queues
//#define MAX_RATE APP_TIMER_TICKS(1, BIKE_TIMER_PRESCALER) //not sure of this
#define BIKE_TIMER_MAX_TICKS        16777216

/*
 *  STATIC VARIABLES
 *****************************************************************************/

static app_timer_id_t millis_counter_timer;


/******************************************************************************
 * FUNCTIONS
 *****************************************************************************/

void timers_init(void);
void timers_start(void);

void millis_counter_handler(void* p_context);
uint32_t get_millis();

#endif /*BIKE_TIMERS_H*/
