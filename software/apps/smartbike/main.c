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
#include "AccelTurnControl.h"
#include "AccelerometerControl.h"
#include "ServoControl.h"
#include "LightControl.h"
#include "BikeState.h"
#include "HallEffectControl.h"
#include "BikeTimers.h"
#include "LightControl.h"
#include "LightAction.h"
#include "PinStatus.h"


/*******************************************************************************
 *   DEFINES
 ******************************************************************************/
//#define DEVICE_NAME "YITIAN"

// Maximum size is 17 characters
#define PHYSWEB_URL     "goo.gl/XMRl3M"

#define GPIOTE_CHANNEL_0 0
#define GPIOTE_CHANNEL_1 1

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

// Light Action
LightAction light_act;

// Accelerometer Data
int16_t curr_x_val = 0;

static bool accel_ready = false;

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

// Timer fired handler
static void timer_handler (void* p_context) {
    //led_toggle(BLEES_LED_PIN);
    // led_toggle(LED_1);
    accel_ready = true;
     
}

static void timer_handler2 (void* p_context) {
    //led_toggle(BLEES_LED_PIN);
    if (bike == NULL) return;
    // led_toggle(LED_2);
    if (bike->blinking_light_output == LIGHT_STATE_BLINKING_OFF) {
        bike->blinking_light_output = LIGHT_STATE_BLINKING_ON;
        // bike->curr_gear = 0;
    } else if (bike->blinking_light_output == LIGHT_STATE_BLINKING_ON) {
        bike->blinking_light_output = LIGHT_STATE_BLINKING_OFF;
        // bike->curr_gear = 2;
    } else { 
        bike->blinking_light_output = LIGHT_STATE_BLINKING_ON;
        // bike->curr_gear = 4;
    }

}

/*******************************************************************************
 *   INIT FUNCTIONS
 ******************************************************************************/

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
    // led_on(LED_0);
}

void ble_evt_disconnected(ble_evt_t* p_ble_evt) {
    // led_off(LED_0);
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
    if (pin == bike->pin_mappings[WHEEL_FLAG]) { 
        led_toggle(LED_2);
        if (bike == NULL) return;

        wheel_interrupt_handler(bike);
        //led_toggle(LED_1);
        /*if (test_milli_count_flag) {
            led_toggle(LED_0);
        }*/
        if (bike->curr_milli > bike->last_milli && milli < bike->curr_milli) {
            //led_toggle(LED_2);
        }
        milli = bike->curr_milli;
        setPinStatus(bike->pin_mappings[WHEEL_FLAG], true);
        //led_toggle(LED_2);
    }
}

void port_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    if (bike == NULL) {
        return;
    }
    /*********************************************************/
    /*   PEDALLING HALL EFFECT INTERRUPT                     */
    /*********************************************************/
    if (pin == bike->pin_mappings[PEDAL_FLAG]) {
        led_toggle(LED_1);
        pedalling_interrupt_handler(bike);
        setPinStatus(bike->pin_mappings[PEDAL_FLAG], true);
    /*********************************************************/
    /*   SHIFT UP BUTTON INTERRUPT                           */
    /*********************************************************/
    } else if (pin == bike->pin_mappings[SHIFT_UP_FLAG]) {
        led_toggle(LED_2);
        setPinStatus(bike->pin_mappings[SHIFT_UP_FLAG], true);
    /*********************************************************/
    /*   SHIFT DOWN BUTTON INTERRUPT                         */
    /*********************************************************/
    } else if (pin == bike->pin_mappings[SHIFT_DOWN_FLAG]) {
        led_toggle(LED_0);   
        // b21 = true;
        setPinStatus(bike->pin_mappings[SHIFT_DOWN_FLAG], true);
    /*********************************************************/
    /*   LEFT TURN BUTTON INTERRUPT                          */
    /*********************************************************/
    } else if (pin == bike->pin_mappings[LEFT_TURN_FLAG]) {
        led_toggle(LED_1); 
        // b22 = true;
        setPinStatus(bike->pin_mappings[LEFT_TURN_FLAG], true);
    /*********************************************************/
    /*   RIGHT TURN BUTTON INTERRUPT                         */
    /*********************************************************/
    } else if (pin == bike->pin_mappings[RIGHT_TURN_FLAG]) {
        led_toggle(LED_2); 
        // b22 = true;
        setPinStatus(bike->pin_mappings[RIGHT_TURN_FLAG], true);
    /*********************************************************/
    /*   MANUAL MODE SWITCH INTERRUPT                        */
    /*********************************************************/
    } else if (pin == bike->pin_mappings[MANUAL_MODE_SWITCH_FLAG]) {
        led_toggle(LED_0); 
        setPinStatus(bike->pin_mappings[MANUAL_MODE_SWITCH_FLAG], true);
    }
    /*********************************************************/
    /*   HALL EFFECT TURN LEFT                               */
    /*********************************************************/
     else if (pin == bike->pin_mappings[HANDLE_LEFT_TURN_FLAG]){
        //update_handle_turn_status(bike, false);
        if (nrf_drv_gpiote_in_is_set(bike->pin_mappings[HANDLE_LEFT_TURN_FLAG])) {
            bike->handle_left_turn = true;
            led_on(LED_0); 
        } else {
            bike->handle_left_turn = false;
            led_off(LED_0); 
        }
        setPinStatus(bike->pin_mappings[HANDLE_LEFT_TURN_FLAG], true);
     }
    /*********************************************************/
    /*   HALL EFFECT TURN RIGHT                              */
    /*********************************************************/
     else if (pin == bike->pin_mappings[HANDLE_RIGHT_TURN_FLAG]){
        // update_handle_turn_status(bike, true);
        if (nrf_drv_gpiote_in_is_set(bike->pin_mappings[HANDLE_RIGHT_TURN_FLAG])) {
            bike->handle_right_turn = true;
            led_on(LED_1); 
        } else {
            bike->handle_right_turn = false;
            led_off(LED_1); 
        }
        setPinStatus(bike->pin_mappings[HANDLE_RIGHT_TURN_FLAG], true);
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
    // Do not remove any of the create_state functions
    // We need them to fix an interrupt bug
    bike = create_state();



/*    bool button09 = false, button10 = false, 
         button06 = false, button05 = false, button04 = false,
         button03 = false;*/
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
    /*                 Initialize GPIO                       */
    /*********************************************************/
    // TODO: change pin polarity and pull configs
    gpio_cfg_t cfgs[] = {
        {bike->pin_mappings[WHEEL_FLAG], GPIO_ACTIVE_HIGH, NRF_GPIO_PIN_NOPULL, &gpiote_handler, PIN_GPIOTE_IN},
        {bike->pin_mappings[PEDAL_FLAG], GPIO_ACTIVE_HIGH, NRF_GPIO_PIN_NOPULL, &port_event_handler, PIN_PORT_IN},
        {bike->pin_mappings[SHIFT_UP_FLAG], GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &port_event_handler, PIN_PORT_IN},
        {bike->pin_mappings[SHIFT_DOWN_FLAG], GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &port_event_handler, PIN_PORT_IN},
        {bike->pin_mappings[LEFT_TURN_FLAG], GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &port_event_handler, PIN_PORT_IN},
        {bike->pin_mappings[RIGHT_TURN_FLAG], GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &port_event_handler, PIN_PORT_IN},
        {bike->pin_mappings[MANUAL_MODE_SWITCH_FLAG], GPIO_ACTIVE_TOGGLE, NRF_GPIO_PIN_NOPULL, &port_event_handler, PIN_PORT_IN},
        {bike->pin_mappings[HANDLE_RIGHT_TURN_FLAG], GPIO_ACTIVE_TOGGLE , NRF_GPIO_PIN_NOPULL, &port_event_handler, PIN_PORT_IN},
        {bike->pin_mappings[HANDLE_LEFT_TURN_FLAG], GPIO_ACTIVE_TOGGLE , NRF_GPIO_PIN_NOPULL, &port_event_handler, PIN_PORT_IN}
    };

    uint8_t gpio_cfg_count;
    gpio_cfg_count = 9;

    err_code = gpio_init(cfgs, gpio_cfg_count);
    APP_ERROR_CHECK(err_code);
 
    gpio_input_enable_all();

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
    set_accel_handler(timer_handler);
    set_turn_signal_handler(timer_handler2);
    timers_init();
    timers_start();
    // TODO: setup timer for ble data out

    /*********************************************************/
    /*               Initialize Accelerometer                */
    /*********************************************************/
    initializeAccelerometer();
    initializeDataBank(true, false, false);

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
    pca9685_init(&twi_instance, PWM0_ADDR);
    pca9685_setPWMFreq(52.0f, PWM0_ADDR);

    pca9685_init(&twi_instance, PWM1_ADDR);
    pca9685_setPWMFreq(52.0f, PWM1_ADDR);

    update_servos(bike);
    initializeLights();
    
    initializePinStatus();
    
    // Do not remove any of the create_state functions
    // We need them to fix an interrupt bug
    destroy_state(bike);
    bike = create_state();
    /*********************************************************/
    /*                     Main Loop                         */
    /*********************************************************/
    while (1) {
        /* GET DATA */
        /*****************************************************/
        /*     Button Press Update                           */
        /*****************************************************/
        // Get the latest button statuses and store locally
        /*button10 = getPinStatusClear(10);
        button09 = getPinStatusClear(9);
        button06 = getPinStatusClear(6);
        button05 = getPinStatusClear(5);
        button04 = getPinStatusClear(4);
        button03 = getPinStatusClear(3);*/
        state_update_flags(bike);

        /*****************************************************/
        /*     Accelerometeter Data Update                   */
        /*****************************************************/
        // Get the latest accelerometer data & store locally
        
        // TODO: Only if Accelerometer is ready
        if (accel_ready) {
            
            populateAccelDataBank();
            accel_ready = false;
        }

        bool newAccelVal = grabAccelData(DATA_X, &curr_x_val, NULL);
// if(readAxisX() <=0){
//     led_on(LED_0);
// }else{
//     led_off(LED_0);
// }

        /* PROCESS DATA */
        /*****************************************************/
        /*     Turn Signal State Machine                     */
        /*****************************************************/
        // Determine what state the turn lights should be in
        /* TODO: UNCOMMENT*/
        btn_state_change_alt(bike);
        
        

        /*****************************************************/
        /*     Turn Signal State Machine                     */
        /*****************************************************/
        /* TODO: UNCOMMENT */
        if(newAccelVal){
            light_act = do_state_action( curr_x_val );
        }

        /*****************************************************/
        /*     Calculate Velocity                            */
        /*****************************************************/
        //velocity and acceleration are updated, target gear set
        if(bike->flags[WHEEL_FLAG] || bike->flags[SHIFT_DOWN_FLAG] ||
           bike->flags[SHIFT_UP_FLAG]) {

            //led_toggle(LED_0);
            update_target_state(bike);
        }


        /* CONTROL HARDWARE */
        /*****************************************************/
        /*     Lighting Control                              */
        /*****************************************************/
        // turn on/off all lights as specified
        
        /* TODO: UNCOMMENT*/
        performLightAction(bike, light_act);

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
        
        // MESSES UP ACCELEROMETER DATA
        //power_manage();
    }

    destroy_state(bike);

    return 0;
}

