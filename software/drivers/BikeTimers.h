#ifndef BIKE_TIMERS_H
#define BIKE_TIMERS_H

#include <stdint.h>

#include "nrf.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "nrf51_bitfields.h"
#include "nrf_drv_config.h"
#include "app_timer_appsh.h"

/******************************************************************************
 *  DEFINES
 *****************************************************************************/

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            4                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4    
#define BIKE_TIMER_PRESCALER        32  // Value of RTC1 PRESCALER Register
#define BIKE_TIMER_MAX_TIMERS       4   // Maximum number of simultaneous timers
#define BIKE_TIMER_OP_QUEUE_SIZE    4   // Size of timer operation queues
//#define MAX_RATE APP_TIMER_TICKS(1, BIKE_TIMER_PRESCALER) //not sure of this
#define BIKE_TIMER_MAX_TICKS        16777216

#define ACCEL_TIMER_UPDATE_RATE           APP_TIMER_TICKS(500, BIKE_TIMER_PRESCALER) // 64Hz => 1/64 ~= 15.6ms; 15ms to get more rather than less data
#define TURN_LIGHT_TIMER_UPDATE_RATE      APP_TIMER_TICKS(500, BIKE_TIMER_PRESCALER) // 2Hz 
/*
 *  STATIC VARIABLES
 *****************************************************************************/

static app_timer_id_t millis_counter_timer;
static app_timer_id_t accel_sample_timer;
static app_timer_id_t turn_signal_sample_timer;
static app_timer_id_t ble_timer;

/******************************************************************************
 * FUNCTIONS
 *****************************************************************************/
void timers_app_init(void);
void timers_init(void);
void timers_start(void);

void millis_counter_handler(void* p_context);
uint32_t get_millis();

void set_accel_handler(app_timer_timeout_handler_t timeout_handler);
void set_turn_signal_handler(app_timer_timeout_handler_t timeout_handler);

#endif /*BIKE_TIMERS_H*/
