/*
 * Eddystone URL advertisement
 */

// Standard Libraries
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// Nordic Libraries
#include "ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "softdevice_handler.h"
#include "nordic_common.h"
// Platform, Peripherals, Devices, Services
#include "nrf_drv_config.h"
#include "smartbike.h"
#include "led.h"
#include "simple_ble.h"
#include "eddystone.h"
#include "simple_adv.h"
#include "ble_config.h"

#include "AccelerometerControl.h"

#define BLEES_LED_PIN LED_0
//#define DEVICE_NAME "YITIAN"


#define BLINK_TIMER_PRESCALER       0   // Value of RTC1 PRESCALER register
#define BLINK_TIMER_MAX_TIMERS      4   // Maximum number of simultaneous timers
#define BLINK_TIMER_OP_QUEUE_SIZE   4   // Size of timer operation queues
#define BLINK_RATE  APP_TIMER_TICKS(1000, BLINK_TIMER_PRESCALER) // Blink every 0.5 seconds

const ble_uuid128_t smartbike_uuid128 = {
    {0x04, 0x08, 0x13, 0x8b, 0x13, 0x02, 0x4e, 0x75,
     0x8c, 0xca, 0xc7, 0x5f, 0x70, 0xdf, 0xf8, 0x9f}
};

ble_uuid_t smartbike_uuid;

// Main application state
simple_ble_app_t* simple_ble_app;
static ble_app_t app;


unsigned char names[] = "ALAN";
unsigned char data[3];

static app_timer_id_t test_timer;


//service error callback
static void service_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

static void sys_evt_dispatch(uint32_t sys_evt) {
    // pstorage_sys_event_handler(sys_evt);
    // on_sys_evt(sys_evt);
}


static void timer_handler (void* p_context) {
    //led_toggle(BLEES_LED_PIN);
    led_toggle(LED_0);
    int16_t xval = readAxisX();
    data[0] = (uint8_t)xval;
    data[1] = (uint8_t)(xval >> 8);
}

static void timers_init(void) {
    uint32_t err_code;

    APP_TIMER_INIT(BLINK_TIMER_PRESCALER, BLINK_TIMER_MAX_TIMERS,
            BLINK_TIMER_OP_QUEUE_SIZE, false);

    err_code = app_timer_create(&test_timer, APP_TIMER_MODE_REPEATED,
            timer_handler);
    APP_ERROR_CHECK(err_code);
}

// Start the timers
static void timers_start(void) {
    uint32_t err_code = app_timer_start(test_timer, BLINK_RATE, NULL);
    APP_ERROR_CHECK(err_code);
}

// Intervals for advertising and connections
static const simple_ble_config_t ble_config = {
    //.platform_id       = PLATFORM_ID_BYTE,  // used as 4th octet in device BLE address
    .platform_id	= 0x81,
    .device_id         = DEVICE_ID_DEFAULT, // 5th and 6th octets in device BLE address
    .adv_name          = DEVICE_NAME,       // used in advertisements if there is room
    .adv_interval      = MSEC_TO_UNITS(500, UNIT_0_625_MS),
    .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
    .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

void services_init() {
    app.service_handle = simple_ble_add_service(&smartbike_uuid128,
						 &smartbike_uuid,
						 0x1816);
    simple_ble_add_characteristic(1, 0, 0,
				  smartbike_uuid.type,
				  0x1817,
				  0x02, &app.test_data,
				  app.service_handle,
				  NULL);
}

// Maximum size is 17 characters
#define PHYSWEB_URL     "goo.gl/XMRl3M"

simple_ble_app_t* simple_ble_app;

void ble_error(uint32_t error_code) {
    led_on(SQUALL_LED_PIN);
}

void ble_evt_connected(ble_evt_t* p_ble_evt) {
    led_on(BLEES_LED_PIN);
}

void ble_evt_disconnected(ble_evt_t* p_ble_evt) {
    led_off(BLEES_LED_PIN);
}

int main(void) {

    // Initialization
    //led_init(SQUALL_LED_PIN);
    led_init(BLEES_LED_PIN);
    //led_on(SQUALL_LED_PIN);
    led_on(BLEES_LED_PIN);

    uint8_t temp_data0 = 0x43;
    uint8_t temp_data1 = 0x44;

    data[0] = (unsigned char)temp_data0;
    data[1] = (unsigned char)temp_data1;
    data[2] = '\n';


    // Setup clock
    //SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, false);

    // Setup and start timer
    timers_init();
    timers_start();

    app.test_data = 0xCDAB;
    simple_ble_app = simple_ble_init(&ble_config);
    //eddystone_adv(PHYSWEB_URL, NULL);
    //simple_adv_only_name();
    simple_adv_service(&smartbike_uuid);
    // Initialization complete
    led_off(SQUALL_LED_PIN);

    initializeAccelerometer();
        

    while (1) {
        power_manage();
    }
}

