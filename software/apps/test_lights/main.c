/*
 * LED Blink App
 */

// Standard Libraries
#include <stdint.h>
#include <stdbool.h>

// Nordic Libraries
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
//#include "boards.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"

// Platform, Peripherals, Devices, Services
#include "smartbike.h"
#include "led.h"
#include "gpio_driver.h"
#include "pwm.h"
#include "AccelerometerControl.h"
#include "AccelDataDriver.h"
#include "AccelTurnControl.h"
#include "BikeState.h"
#include "LightControl.h"
#include "LightAction.h"
#include "PinStatus.h"
#include "BikeTimers.h"

/*******************************************************************************
 *   DEFINES
 ******************************************************************************/
#include "nrf_drv_config.h"

#define BLINK_TIMER_PRESCALER       0   // Value of RTC1 PRESCALER register
#define BLINK_TIMER_MAX_TIMERS      4   // Maximum number of simultaneous timers
#define BLINK_TIMER_OP_QUEUE_SIZE   4   // Size of timer operation queues
#define BLINK_RATE  APP_TIMER_TICKS(500, BLINK_TIMER_PRESCALER) // Blink every 0.5 seconds

#define GPIOTE_CHANNEL_0 0
#define GPIOTE_CHANNEL_1 1

/*******************************************************************************
 *   STATIC AND GLOBAL VARIABLES
 ******************************************************************************/

static app_timer_id_t test_timer;
LightAction light_act;
int16_t curr_x_val = 0;
nrf_drv_twi_t twi_instance = NRF_DRV_TWI_INSTANCE(1);

static bool accel_ready = false;
State* bike;
/*******************************************************************************
 *   HANDLERS AND CALLBACKS
 ******************************************************************************/

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name) {
    // APPL_LOG("[APPL]: ASSERT: %s, %d, error 0x%08x\r\n", p_file_name, line_num, error_code);
    // nrf_gpio_pin_set(ASSERT_LED_PIN_NO);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}

/*@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

//service error callback
static void service_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt) {
    // pstorage_sys_event_handler(sys_evt);
    // on_sys_evt(sys_evt);
}

// Timer fired handler
static void timer_handler (void* p_context) {
    //led_toggle(BLEES_LED_PIN);
    led_toggle(LED_0);
    accel_ready = true;
     
}

static void timer_handler2 (void* p_context) {
    //led_toggle(BLEES_LED_PIN);
    if (bike == NULL) return;
    led_toggle(LED_2);
    if (bike->blinking_light_output == LIGHT_STATE_BLINKING_OFF) {
        bike->blinking_light_output == LIGHT_STATE_BLINKING_ON;
    } else if (bike->blinking_light_output == LIGHT_STATE_BLINKING_ON) {
        bike->blinking_light_output == LIGHT_STATE_BLINKING_OFF;
    } else { 
        bike->blinking_light_output == LIGHT_STATE_BLINKING_ON;
    }

}

/*******************************************************************************
 *   INIT FUNCTIONS
 ******************************************************************************/

/*static void timers_init(void) {
    uint32_t err_code;

    APP_TIMER_INIT(BLINK_TIMER_PRESCALER, BLINK_TIMER_MAX_TIMERS,
            BLINK_TIMER_OP_QUEUE_SIZE, false);

    err_code = app_timer_create(&test_timer, APP_TIMER_MODE_REPEATED,
            timer_handler);
    APP_ERROR_CHECK(err_code);
}*/

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
 *   HELPER FUNCTIONS
 ******************************************************************************/

/** @brief Function for the Power manager.
*/
static void power_manage (void) {
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

// Start the timers
/*static void timers_start(void) {
    uint32_t err_code = app_timer_start(test_timer, BLINK_RATE, NULL);
    APP_ERROR_CHECK(err_code);
}*/


/*******************************************************************************
 *   MAIN LOOP
 ******************************************************************************/

#define BUTTON_PIN 21
#define BUTTON2_PIN 22

#define PIN1 8 
#define PIN2 9
#define ACCEL_PIN 10
// static bool accelDataReady = false;
// volatile int16_t sampled_accel_x = 0;
// static const int16_t sampled_accel_x = 0;
// volatile uint16_t thresh_out_count = 0;
// volatile uint16_t thresh_in_count = 0;

#define ACCEL_THRESH 150

#define ACCEL_OUT_THRESH 200
#define ACCEL_IN_THRESH 150
#define ACCEL_TILT_THRESH 100
// Interrupt handler
/*void GPIOTE_IRQHandler(){
//led_toggle(LED_0);
nrf_gpio_pin_toggle(LED_0);
//nrf_gpio_pin_toggle(OUTPUT_PIN);
NRF_GPIOTE->EVENTS_IN[0] = 0;
}*/


volatile bool b8, b9, b10, b21, b22, leftSignal, rightSignal;


void pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    // if (pin == 8) {
    //     b8 = true;
    // } else if (pin == 9) {
    //     b9 = true;
    //if (pin == 10) { // accelerometer interrupt
    //  accelDataReady = true;
    //}else 
    if (pin == 9) { //left button interrupt
        // led_on(LED_0);
        b21 = true;
        // btn_state_change(b21, b22);
        // light_act = do_state_action( curr_x_val );
        // b21 = false;
        // led_off(LED_0);
        setPinStatus(9, true);
        led_toggle(LED_0);
    }
    else if (pin == 10) { // right btn
        //b22 = true;
        // led_toggle(LED_2);
        // led_on(LED_1);
        b22 = true;
        // btn_state_change(b21, b22);
        // light_act = do_state_action( curr_x_val );
        // b22 = false;
        // led_off(LED_1);
        setPinStatus(10, true);
        led_toggle(LED_2);

    }  else if (pin == 24) { //hall effect turn update left turn
        // led_toggle(LED_0);
        //update_handle_turn_status(bike, false);
        if (nrf_drv_gpiote_in_is_set(bike->pin_mappings[HANDLE_LEFT_TURN_FLAG])) {
            bike->handle_left_turn = true;
            led_on(LED_0); 
        } else {
            bike->handle_left_turn = false;
            led_off(LED_0); 
        }
        setPinStatus(24, true); //hall effect turn update  right turn
    } else if (pin == 8) {
        // led_toggle(LED_1);
        // update_handle_turn_status(bike, true);
        if (nrf_drv_gpiote_in_is_set(bike->pin_mappings[HANDLE_RIGHT_TURN_FLAG])) {
            bike->handle_right_turn = true;
            led_on(LED_1); 
        } else {
            bike->handle_right_turn = false;
            led_off(LED_1); 
        }
        setPinStatus(8, true);
    }

}

int main(void) {
    // LightAction light_act;

    uint32_t err_code;
    uint8_t gpio_count;
    // // Initialization
    b8=false; b9=false; b10=false; b21=false; b22=false, leftSignal = false; rightSignal = false;
    led_init(LED_0);
    led_init(LED_1);
    led_init(LED_2);


    static gpio_cfg_t cfgs[] = {  {9, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler, PIN_PORT_IN},
                                  {10, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler, PIN_PORT_IN},
                                 {8, GPIO_ACTIVE_TOGGLE , NRF_GPIO_PIN_NOPULL, &pin_handler, PIN_PORT_IN},
                                  {24, GPIO_ACTIVE_TOGGLE , NRF_GPIO_PIN_NOPULL, &pin_handler, PIN_PORT_IN}};
    gpio_count = 4;

    err_code = gpio_init(cfgs, gpio_count);
    while (err_code) {
        led_on(LED_1);
    }
    gpio_input_enable_all();

    initializeAccelerometer();
    i2c_init();	
    pca9685_init(&twi_instance, LED_LIGHT_PWM_ADDR);
    pca9685_setPWMFreq(52.0f, LED_LIGHT_PWM_ADDR);

    pca9685_init(&twi_instance, REAR_LIGHT_PWM_ADDR);
    pca9685_setPWMFreq(52.0f, REAR_LIGHT_PWM_ADDR);

    initializeLights();

    initializeDataBank(true, false, false);


    // TIMERS
    set_accel_handler(timer_handler);
    set_turn_signal_handler(timer_handler2);
    timers_init();
    timers_start();

    initializePinStatus();
    bike = create_state();
    bike->curr_gear = 7;

    bike->curr_delta = 10;

    // Uncomment for braking, comment out for not braking
    // bike->curr_delta = 0;

    bike->last_delta = 10;
    bike->blinking_light_output = LIGHT_STATE_BLINKING_ON;

    while (true) {
        if (accel_ready) {
            populateAccelDataBank();
            accel_ready = false;
        }
        state_update_flags(bike);
        bool newAccelVal = grabAccelData(DATA_X, &curr_x_val, NULL);

        btn_state_change_alt(bike);

        if(newAccelVal){
            light_act = do_state_action( curr_x_val );
        }
        performLightAction(bike, light_act);
        //setLEDLightState(0, LIGHT_STATE_ON);

        power_manage();
    }
}

