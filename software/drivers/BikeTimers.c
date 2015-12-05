/*
 *  Bike Timers
 */

#include "BikeTimers.h"


app_timer_timeout_handler_t accel_sample_handler;


void timers_init(void) {
    uint32_t err_code;
    
    APP_TIMER_INIT(BIKE_TIMER_PRESCALER, BIKE_TIMER_MAX_TIMERS, 
            BIKE_TIMER_OP_QUEUE_SIZE, false);
    
    // ADD YOUR TIMERS HERE
    err_code = app_timer_create(&millis_counter_timer, 
								APP_TIMER_MODE_REPEATED,
            					millis_counter_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&accel_sample_timer,
                                APP_TIMER_MODE_REPEATED,
                                accel_sample_handler);
    APP_ERROR_CHECK(err_code);
}


void timers_start(void) {
    // ADD YOUR TIMERS HERE
    uint32_t err_code = app_timer_start(millis_counter_timer, BIKE_TIMER_MAX_TICKS, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(accel_sample_timer, ACCEL_TIMER_UPDATE_RATE, NULL);
    APP_ERROR_CHECK(err_code);
}


void millis_counter_handler(void* p_context) {
    // do nothing, just a place holder b/c of the stupid APP TIMER API
}


uint32_t get_millis() {
    return (uint32_t)NRF_RTC1->COUNTER;
}



void set_accel_handler(app_timer_timeout_handler_t timeout_handler) {
    accel_sample_handler = timeout_handler;
}
