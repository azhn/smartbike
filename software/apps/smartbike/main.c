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

// Drivers
#include "gpio_driver.h"
#include "AccelDataDriver.h"
#include "AccelerometerControl.h"
#include "ServoControl.h"
#include "LightControl.h"
#include "BikeState.h"
#include "HallEffectControl.h"
#include "BikeTimers.h"
#include "LightControl.h"
#include "LightAction.h"


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

#define HALL_EFFECT_WHEEL_PIN 6  // GPIOTE
#define HALL_EFFECT_PEDAL_PIN 5  // PORT
#define BUTTON_SHIFT_UP_PIN 4    // PORT
#define BUTTON_SHIFT_DOWN_PIN 3  // PORT
#define BUTTON_LEFT_TURN_PIN 9 // PORT
#define BUTTON_RIGHT_TURN_PIN 10   // PORT



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

// GPIO
uint32_t pin_status = 0x0000;

// Accelerometer Data
int16_t curr_x_val = 0;


/*******************************************************************************
 *   FUNCTION DECLARATIONS
 ******************************************************************************/


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



/*******************************************************************************
 *   INIT FUNCTIONS
 ******************************************************************************/


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

//setup i2c
void i2c_init(void)
{
    nrf_drv_twi_config_t twi_config;

    twi_config.sda = I2C_SDA_PIN;
    twi_config.scl = I2C_SCL_PIN;
    twi_config.frequency = NRF_TWI_FREQ_400K;
    twi_config.interrupt_priority = 2;

    nrf_drv_twi_init(&twi_instance, &twi_config, NULL);
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
}

/*******************************************************************************
*   INTERRUPT HANDLER
******************************************************************************/
void gpiote_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    static uint32_t milli = 0;
    if (bike == NULL) {
        return;
    }
    /*********************************************************/
    /*   Wheel Hall Effect Interrupt                         */
    /*********************************************************/
    if (pin == HALL_EFFECT_WHEEL_PIN) { 
        if (bike == NULL) return;

        wheel_interrupt_handler(bike);
        //led_toggle(LED_1);
        /*if (test_milli_count_flag) {
            led_toggle(LED_0);
        }*/
        if (bike->curr_milli > bike->last_milli && milli < bike->curr_milli) {
            led_toggle(LED_2);
        }
        milli = bike->curr_milli;
        setPinStatus(HALL_EFFECT_WHEEL_PIN, true);
        //led_toggle(LED_2);
    }
}

void port_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    /*********************************************************/
    /*   PEDALLING HALL EFFECT INTERRUPT                     */
    /*********************************************************/
    if (pin == HALL_EFFECT_PEDAL_PIN) {
        pedalling_interrupt_handler(bike);
        setPinStatus(HALL_EFFECT_PEDAL_PIN, true);
    /*********************************************************/
    /*   SHIFT UP BUTTON INTERRUPT                           */
    /*********************************************************/
    } else if (pin == BUTTON_SHIFT_UP_PIN) {
        // led_toggle(LED_2);
        setPinStatus(BUTTON_SHIFT_UP_PIN, true);
    /*********************************************************/
    /*   SHIFT DOWN BUTTON INTERRUPT                         */
    /*********************************************************/
    } else if (pin == BUTTON_SHIFT_DOWN_PIN) {
        // led_toggle(LED_0);   
        // b21 = true;
        setPinStatus(BUTTON_SHIFT_DOWN_PIN, true);
    /*********************************************************/
    /*   LEFT TURN BUTTON INTERRUPT                          */
    /*********************************************************/
    } else if (pin == BUTTON_LEFT_TURN_PIN) {
        // led_toggle(LED_1); 
        // b22 = true;
        setPinStatus(BUTTON_LEFT_TURN_PIN, true);
    /*********************************************************/
    /*   RIGHT TURN BUTTON INTERRUPT                         */
    /*********************************************************/
    } else if (pin == BUTTON_RIGHT_TURN_PIN) {
        // led_toggle(LED_1); 
        // b22 = true;
        setPinStatus(BUTTON_RIGHT_TURN_PIN, true);
    }
}

/*******************************************************************************
 *   MAIN FUNCTION
 ******************************************************************************/
int main(void) {
    /*********************************************************/
    /*               Local Variables/Data                    */
    /*********************************************************/
    uint32_t err_code;

    //create our state
    bike = create_state();


    bool button09 = false, button10 = false, 
         button06 = false, button05 = false, button04 = false,
         button03 = false;
    /*********************************************************/
    /*                 Initialize GPIO                       */
    /*********************************************************/
    // TODO: Add a pin for manual/automatic switch
    gpio_cfg_t cfgs[] = {
        {bike->pin_mappings[WHEEL_FLAG], GPIO_ACTIVE_HIGH, NRF_GPIO_PIN_PULLDOWN, &gpiote_handler, PIN_GPIOTE_IN},
        {bike->pin_mappings[PEDAL_FLAG], GPIO_ACTIVE_HIGH, NRF_GPIO_PIN_PULLDOWN, &port_event_handler, PIN_PORT_IN},
        {bike->pin_mappings[SHIFT_UP_FLAG], GPIO_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, &port_event_handler, PIN_PORT_IN},
        {bike->pin_mappings[SHIFT_DOWN_FLAG], GPIO_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, &port_event_handler, PIN_PORT_IN},
        {bike->pin_mappings[LEFT_TURN_FLAG], GPIO_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, &port_event_handler, PIN_PORT_IN},
        {bike->pin_mappings[RIGHT_TURN_FLAG], GPIO_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, &port_event_handler, PIN_PORT_IN},
        {bike->pin_mappings[MANUAL_MODE_SWITCH_FLAG], GPIO_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, &port_event_handler, PIN_PORT_IN}
    };

    uint8_t gpio_cfg_count;
    gpio_cfg_count = 6;

    err_code = gpio_init(cfgs, gpio_cfg_count);
    APP_ERROR_CHECK(err_code);
 
    gpio_input_enable_all();


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
    /*                  Initialize BLE                       */
    /*********************************************************/
/*    app.test_data = 0xCDAB;
    simple_ble_app = simple_ble_init(&ble_config);
    //eddystone_adv(PHYSWEB_URL, NULL);
    //simple_adv_only_name();
    simple_adv_service(&smartbike_uuid);
    // Initialization complete
    //led_on(LED_2);
*/
    /*********************************************************/
    /*                  Initialize Timers                    */
    /*********************************************************/
    // Setup clock
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION, false);

    // Setup and start timer
    timers_init();
    timers_start();
    // TODO: setup timer for ble data out

    /*********************************************************/
    /*               Initialize Accelerometer                */
    /*********************************************************/
//    initializeAccelerometer();

    /*********************************************************/
    /*               Initialize Hall Effects                 */
    /*********************************************************/

    /*********************************************************/
    /*         Initialize PWM Driver Related Outputs         */
    /*                   Servos & Lights                     */
    /*********************************************************/
    //init i2c
    i2c_init();	

    //Setup and init PWM
    pca9685_init(&twi_instance, LED_LIGHT_PWM_ADDR);
    pca9685_setPWMFreq(52.0f, LED_LIGHT_PWM_ADDR);

    pca9685_init(&twi_instance, REAR_LIGHT_PWM_ADDR);
    pca9685_setPWMFreq(52.0f, REAR_LIGHT_PWM_ADDR);

    //pca9685_setPWM(0,0,2000);
    update_servos(bike);
    initializeLights();

    /*********************************************************/
    /*                     Main Loop                         */
    /*********************************************************/
    while (1) {
        /* GET DATA */
        /*****************************************************/
        /*     Button Press Update                           */
        /*****************************************************/
        // Get the latest button statuses and store locally
        button10 = getPinStatusClear(10);
        button09 = getPinStatusClear(9);
        button06 = getPinStatusClear(6);
        button05 = getPinStatusClear(5);
        button04 = getPinStatusClear(4);
        button03 = getPinStatusClear(3);

        /*****************************************************/
        /*     Accelerometeter Data Update                   */
        /*****************************************************/
        // Get the latest accelerometer data & store locally
        
        // TODO: Only if Accelerometer is ready
        populateAccelDataBank();

        bool newAccelVal = grabAccelData(DATA_X, &curr_x_val, NULL);

        /* PROCESS DATA */
        /*****************************************************/
        /*     Turn Signal State Machine                     */
        /*****************************************************/
        // Determine what state the turn lights should be in
        /* TODO: UNCOMMENT
        btn_state_change(button09, button10);
        */ 

        /*****************************************************/
        /*     Turn Signal State Machine                     */
        /*****************************************************/
        /* TODO: UNCOMMENT
        if(newAccelVal){
            light_act = do_state_action( curr_x_val );
        }
        */

        /*****************************************************/
        /*     Calculate Velocity                            */
        /*****************************************************/
        //velocity and acceleration are updated, target gear set
        if(bike->flags[WHEEL_FLAG])
        {
            led_toggle(LED_0);
            update_target_state(bike);
        }


        /* CONTROL HARDWARE */
        /*****************************************************/
        /*     Lighting Control                              */
        /*****************************************************/
        // turn on/off all lights as specified
        
        /* TODO: UNCOMMENT
        performLightAction(bike, light_act);
        */

        /*****************************************************/
        /*     Shifting Control                              */
        /*****************************************************/
        /*if(bike->flags[PEDAL_FLAG]) {
            update_servos(bike);
        }*/

        update_servos(bike);
        /*****************************************************/
        /*     Manage power (do we need this?)               */
        /*****************************************************/
        power_manage();
    }

	destroy_state(bike);

	return 0;
}

