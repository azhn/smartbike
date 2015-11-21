/*
 * Smartbike
 *
 *  Dinker Ambe, Charles Hardin, Timothy Kenny, Kunjan Singh, Alan Zhen
 */

/*******************************************************************************
 *   INCLUDES
 ******************************************************************************/
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
// #include "simple_adv.h"
#include "ble_config.h"

#include "AccelerometerControl.h"
#include "ServoControl.h"
#include "LightControl.h"
#include "BikeState.h"


/*******************************************************************************
 *   DEFINES
 ******************************************************************************/
//#define DEVICE_NAME "YITIAN"

// Maximum size is 17 characters
#define PHYSWEB_URL     "goo.gl/XMRl3M"

#define GPIOTE_CHANNEL_0 0
#define GPIOTE_CHANNEL_1 1

// GPIO PINS
#define PIN_08 8
#define PIN_09 9
#define PIN_10 10
#define PIN_21 21
#define PIN_22 22

// # defines from Blink app
#define BLINK_TIMER_PRESCALER       0   // Value of RTC1 PRESCALER register
#define BLINK_TIMER_MAX_TIMERS      4   // Maximum number of simultaneous timers
#define BLINK_TIMER_OP_QUEUE_SIZE   4   // Size of timer operation queues
#define BLINK_RATE  APP_TIMER_TICKS(1000, BLINK_TIMER_PRESCALER) // Blink every 0.5 seconds

#define NUM_GEARS 6
#define MM_PER_INT 1100UL

/*******************************************************************************
 *   CONSTANTS
 ******************************************************************************/
const ble_uuid128_t smartbike_uuid128 = {
    {0x04, 0x08, 0x13, 0x8b, 0x13, 0x02, 0x4e, 0x75,
     0x8c, 0xca, 0xc7, 0x5f, 0x70, 0xdf, 0xf8, 0x9f}
};

//our bike
State* bike;

//i2c instance 
nrf_drv_twi_t twi_instance = NRF_DRV_TWI_INSTANCE(1);


/*******************************************************************************
 *   GLOBAL VARIABLES
 ******************************************************************************/
// BLE
ble_uuid_t smartbike_uuid;
simple_ble_app_t* simple_ble_app;
static ble_app_t app;
unsigned char names[] = "ALAN";
unsigned char data[3];

// GPIO
uint32_t pin_status = 0x0000;

// HALL EFFECT (wheel)
uint32_t curr_wheel_int_time = 0;
uint32_t prev_wheel_int_time = 0;

// HALL EFFECT (pedal)
uint32_t curr_pedal_int_time = 0;
uint32_t prev_pedal_int_time = 0;


/*******************************************************************************
 *   FUNCTION DECLARATIONS
 ******************************************************************************/
static app_timer_id_t test_timer;


/*******************************************************************************
 *   FUNCTION DEFINITIONS
 ******************************************************************************/
//service error callback
static void service_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

static void sys_evt_dispatch(uint32_t sys_evt) {
    // pstorage_sys_event_handler(sys_evt);
    // on_sys_evt(sys_evt);
}

static void timer_handler (void* p_context) {
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

/*******************************************************************************
 *   INIT FUNCTIONS
 ******************************************************************************/


// Start the timers
static void timers_start(void) {
    uint32_t err_code = app_timer_start(test_timer, BLINK_RATE, NULL);
    APP_ERROR_CHECK(err_code);
}

static void setPinStatus(uint8_t pin_num, bool value){
    if(value) { // set value to 1
        pin_status |= (1<<pin_num);
    } else { // set value to 0
        pin_status &= ( ~(1<<pin_num) );
    }
}

static bool getPinStatus(uint8_t pin_num){
    return ( (pin_status & 1<<pin_num) );
}

static bool getPinStatusClear(uint8_t pin_num){
    bool ret = ( (pin_status & 1<<pin_num) );
    pin_status &= ( ~(1<<pin_num) );
    return ret;
}

/*******************************************************************************
// Intervals for advertising and connections
 ******************************************************************************/
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

simple_ble_app_t* simple_ble_app;

void ble_error(uint32_t error_code) {
    led_on(LED_1);
}

void ble_evt_connected(ble_evt_t* p_ble_evt) {
    led_on(LED_0);
}

void ble_evt_disconnected(ble_evt_t* p_ble_evt) {
    led_off(LED_0);

//setup i2c
static void i2c_init(void)
{
    nrf_drv_twi_config_t twi_config;

    twi_config.sda = I2C_SDA_PIN;
    twi_config.scl = I2C_SCL_PIN;
    twi_config.frequency = NRF_TWI_FREQ_400K;
    twi_config.interrupt_priority = 2;

    nrf_drv_twi_init(&twi_instance, &twi_config, NULL);
}

/*******************************************************************************
*   INTERRUPT HANDLER
******************************************************************************/

void pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    /*********************************************************/
    /*   Pedalling Hall Effect Interrupt                     */
    /*********************************************************/
    if (pin == PIN_08) {
        // led_toggle(LED_1);
        // led_toggle(LED_2);
        // b8 = true;
		wheel_interrupt_handler(bike);
        setPinStatus(PIN_08, true);
    /*********************************************************/
    /*   Wheel Hall Effect Interrupt                         */
    /*********************************************************/
    } else if (pin == PIN_09) { 
        // led_toggle(LED_0);
        // led_toggle(LED_1);
        setPinStatus(PIN_09, true);
    /*********************************************************/
    /*   Accelerometer Interrupt                             */
    /*********************************************************/
    } else if (pin == PIN_10) {
        // led_toggle(LED_2);
        setPinStatus(PIN_10, true);
    /*********************************************************/
    /*   Left Button Interrupt                               */
    /*********************************************************/
    }else if (pin == PIN_21) {
        // led_toggle(LED_0);   
        // b21 = true;
        setPinStatus(PIN_21, true);
    }
    /*********************************************************/
    /*   Right Button Interrupt                              */
    /*********************************************************/
    else if (pin == PIN_22) {
        // led_toggle(LED_1); 
        // b22 = true;
        setPinStatus(PIN_22, true);
    }
}

/*******************************************************************************
 *   MAIN FUNCTION
 ******************************************************************************/
int main(void) {
    /*********************************************************/
    /*               Local Variables/Data                    */
    /*********************************************************/
	//create our state
	bike = create_state();
    
	nrf_drv_gpiote_in_config_t temp_t = {NRF_GPIOTE_POLARITY_HITOLO, NRF_GPIO_PIN_NOPULL, false, false};

    uint8_t temp_data0 = 0x43;
    uint8_t temp_data1 = 0x44;

    data[0] = (unsigned char)temp_data0;
    data[1] = (unsigned char)temp_data1;
    data[2] = '\n';

    bool button08 = false, button09 = false, button10 = false, 
        button21 = false, button22 = false;

    /*********************************************************/
    /*                  Initialize Lights                    */
    /*********************************************************/
    // Initialization of LEDs (two methods)
    led_init(LED_0);
    led_init(LED_1);
    led_init(LED_2);
    //  OR
    // nrf_gpio_cfg_output(LED_0);  //Configure LED 0 as output
    // nrf_gpio_cfg_output(LED_1);  //Configure LED 1 as output


    /*********************************************************/
    /*                  Initialize Timers                    */
    /*********************************************************/
    // Setup clock
    //SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, false);

    // Setup and start timer
    timers_init();
    timers_start();
        // TODO: setup timer for ble data out


    /*********************************************************/
    /*                  Initialize BLE                       */
    /*********************************************************/
    app.test_data = 0xCDAB;
    simple_ble_app = simple_ble_init(&ble_config);
    //eddystone_adv(PHYSWEB_URL, NULL);
    //simple_adv_only_name();
    simple_adv_service(&smartbike_uuid);
    // Initialization complete
    led_off(LED_2);


    /*********************************************************/
    /*               Initialize Accelerometer                */
    /*********************************************************/
    initializeAccelerometer();

    /*********************************************************/
    /*               Initialize Hall Effects                 */
    /*********************************************************/

    /*********************************************************/
    /*                    Initialize Servos                  */
    /*********************************************************/
	//init i2c
	i2c_init();	
	
	//Setup and init PWM
	pca9685_init(&twi_instance);
	pca9685_setPWMFreq(52.0f);
	update_servos(bike);

    /*********************************************************/
    /*                 Initialize GPIO                       */
    /*********************************************************/
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    nrf_drv_gpiote_in_init(21, &temp_t, &pin_handler);
    nrf_gpio_cfg_sense_input(21, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    

    nrf_drv_gpiote_in_init(22, &temp_t, &pin_handler);    
    nrf_gpio_cfg_sense_input(22, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);

    nrf_drv_gpiote_in_init(8, &temp_t, &pin_handler);
    nrf_gpio_cfg_sense_input(8, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);

    nrf_drv_gpiote_in_init(9, &temp_t, &pin_handler);
    nrf_gpio_cfg_sense_input(9, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);

    nrf_drv_gpiote_in_init(10, &temp_t, &pin_handler);
    nrf_gpio_cfg_sense_input(10, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);

    // nrf_drv_gpiote_in_init(0, &temp_t, &pin_handler);
    NRF_GPIOTE->INTENSET = 0x8000000F;
    NVIC_EnableIRQ(GPIOTE_IRQn);


    /*********************************************************/
    /*                     Main Loop                         */
    /*********************************************************/
    while (1) {
        /* GET DATA */
        /*****************************************************/
        /*     Button Press Update                           */
        /*****************************************************/
        // Get the latest button statuses and store locally
        button08 = getPinStatusClear(8);
        button09 = getPinStatusClear(9);
        button10 = getPinStatusClear(10);
        button21 = getPinStatusClear(21);
        button22 = getPinStatusClear(22);

        /*****************************************************/
        /*     Accelerometeter Data Update                   */
        /*****************************************************/
        // Get the latest accelerometer data & store locally
        
        /*****************************************************/
        /*     Hall Effect Pedalling Update                  */
        /*****************************************************/
        // Only change gears if pedalling forward at a
        //    decent speed

        /*****************************************************/
        /*     Hall Effect Wheel Speed Update                */
        /*****************************************************/
        // Compare new hall effect interrupt with preivous
        //    to calculate (wheel) speed




        /* PROCESS DATA */
        /*****************************************************/
        /*     Turn Signal State Machine                     */
        /*****************************************************/
        // Determine what state the turn lights should be in

        /*****************************************************/
        /*     Turn Signal State Machine                     */
        /*****************************************************/
        // Determine what gear we should shift to based on
        //  wheel speed and accelerometer tilt (slope)


        
        /* CONTROL HARDWARE */
        /*****************************************************/
        /*     Lighting Control                              */
        /*****************************************************/
        // turn on/off all lights as specified

        /*****************************************************/
        /*     Shifting Control                              */
        /*****************************************************/
        // shift to the correct gear

        /*****************************************************/
        /*     Manage power (do we need this?)               */
        /*****************************************************/
        power_manage();
    }

	destroy_state(bike);

	return 0;
}

