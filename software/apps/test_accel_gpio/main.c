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
#include "AccelerometerControl.h"


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
    led_toggle(LED_0);
}


/*******************************************************************************
 *   INIT FUNCTIONS
 ******************************************************************************/

static void timers_init(void) {
    uint32_t err_code;

    APP_TIMER_INIT(BLINK_TIMER_PRESCALER, BLINK_TIMER_MAX_TIMERS,
            BLINK_TIMER_OP_QUEUE_SIZE, false);

    err_code = app_timer_create(&test_timer, APP_TIMER_MODE_REPEATED,
            timer_handler);
    APP_ERROR_CHECK(err_code);
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
static void timers_start(void) {
    uint32_t err_code = app_timer_start(test_timer, BLINK_RATE, NULL);
    APP_ERROR_CHECK(err_code);
}


/*******************************************************************************
 *   MAIN LOOP
 ******************************************************************************/

#define BUTTON_PIN 21

#define BUTTON2_PIN 22

#define PIN1 8 
#define PIN2 9
#define ACCEL_PIN 10
static volatile uint32_t i = 0;
static bool accelDataReady = false;
//volatile int16_t sampled_accel_x = 0;
static const int16_t sampled_accel_x = 0;
volatile int16_t curr_x_total = 0;
volatile uint16_t thresh_out_count = 0;
volatile uint16_t thresh_in_count = 0;

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
volatile bool checkForReturn, returning;

void pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    // if (pin == 8) {
    //     b8 = true;
    // } else if (pin == 9) {
    //     b9 = true;
    if (pin == 10) { // accelerometer interrupt
         accelDataReady = true;
    }else if (pin == 21) { //button interrupt
        b21 = true;
    }
    else if (pin == 22) { 
         //b22 = true;
        b22 = true;
    }    
}

// checks if the accelerometer is within the threshold of the sampled value
bool check_accel_x(){
    // while(1) {
    led_on(LED_2);
        if(accelDataReady){

            if(abs(readAxisX() - sampled_accel_x) <= ACCEL_THRESH){
                led_off(LED_0);
            }
            accelDataReady = false;
        }
        // reset the data ready interrupt by reading an axis reg
        readAxisX();

        led_off(LED_2);
    // }
}

int main(void) {
    uint32_t err_code;
    uint8_t gpio_input_count;
    // // Initialization
    volatile uint32_t time_cnt = 0;
    uint32_t i;
    i=0;
    b8=false; b9=false; b10=false; b21=false; b22=false, leftSignal = false; rightSignal = false;
    checkForReturn = false, returning = false;
    led_init(LED_0);
    led_init(LED_1);
    led_init(LED_2);

    //initializeAccelerometer();
    initializeAccelerometer();

    // since active high, pins need to be set to have a pull-down resistor,
    //      otherwise they will be floating
    static gpio_input_cfg_t cfgs[] = {  {BUTTON_PIN, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler},
                                        {BUTTON2_PIN, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler},
                            
    //                                     {PIN1, GPIO_ACTIVE_HIGH, NRF_GPIO_PIN_NOPULL, &pin_handler},
    //                                     {PIN2, GPIO_ACTIVE_HIGH, NRF_GPIO_PIN_NOPULL, &pin_handler},
                                        {ACCEL_PIN, GPIO_ACTIVE_HIGH, NRF_GPIO_PIN_NOPULL, &pin_handler}};
    // // It seems only 4 pins can be registered per channel
    gpio_input_count = 3;

    /* SET OUTPUT WITH DRIVER */
    /* uint8_t output_pins[] = {PIN1,PIN2,PIN3};
     * gpio_output_init(output_pins, 3);
     */

    /* SET OUTPUT OLD WAY */
    /* nrf_gpio_cfg_output(OUTPUT_PIN);
     */

    
    /* SET INPUT WITH DRIVER */
    err_code = gpio_input_init(cfgs, gpio_input_count);
    if (err_code) {
        led_on(LED_1);
    }
    gpio_input_enable_all();

    /* SET INPUT OLD WAY */
    /* nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_NOPULL); //Configure pin 21 0 as input
     * nrf_gpiote_event_configure(GPIOTE_CHANNEL_0, BUTTON_PIN, NRF_GPIOTE_POLARITY_LOTOHI); 
     * nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);
     */ 

    //NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled; //Set GPIOTE interrupt register on channel 0
    NVIC_EnableIRQ(GPIOTE_IRQn); //Enable interrupts


//accelDataReady = true;
    // reset accelerometer data ready interrupt
    readAxisX();
    while (true) {

        /*******************************************************************
        Working Accel turn light
        ********************************************************************/
        /*
        if (b21 == true) { //button was toggled
            
            led_on(LED_0);
            // need to sample the accelerometer value
            sampled_accel_x = readAxisX();
            //accelDataReady = false;
            
            checkForReturn = true;
            
            b21 = false;

            for(time_cnt = 0; time_cnt < 1000000; time_cnt++){
                //do nothing, jsut to delay
            }
        }

        // if(checkForReturn){
        while(checkForReturn){

            if(abs(readAxisX() - sampled_accel_x) <= ACCEL_THRESH){
                led_off(LED_0);
                checkForReturn = false;
            }
            // accelDataReady = false;
        }
        // reset the data ready interrupt by reading an axis reg
        readAxisX();
        */
        /*******************************************************************
        ********************************************************************/

        if (b21 == true) { //button was toggled
            // need to sample the accelerometer value
            //  1) add new sample to sample value
            //sampled_accel_x = readAxisX();
            //sampled_accel_x = 0;
            
            // turn on LED due to button press (signal light)
            
            // check for return to ~0
            
            
            // turn off button called
            b21 = false;
            
            if (checkForReturn && !leftSignal) {
                led_off(LED_1);
            }
            leftSignal = true;
            checkForReturn = true;
            led_on(LED_0);
            // for(time_cnt = 0; time_cnt < 1000000; time_cnt++){
            //     //do nothing, jsut to delay
            // }
        }

        if (b22 == true) { //button was toggled
            // need to sample the accelerometer value
            //  1) add new sample to sample value
            //sampled_accel_x = readAxisX();
            //sampled_accel_x = 0;
            
            // turn on LED due to button press (signal light)
            
            // check for return to ~0
            
            // turn off button called
            b22 = false;
            if (checkForReturn && leftSignal) {
                led_off(LED_0);
            }
            leftSignal = false;
            checkForReturn = true;

            led_on(LED_1);

            // for(time_cnt = 0; time_cnt < 1000000; time_cnt++){
            //     //do nothing, jsut to delay
            // }
        }

         if(checkForReturn && accelDataReady){
            int16_t curr_x_val = readAxisX();
            // add current sample to total
            // if( leftSignal && (curr_x_val < 0)){
            //     curr_x_total += curr_x_val;    
            // } else if( !leftSignal && (curr_x_val > 0)){
            //     curr_x_total += curr_x_val;
            // }
            
        
            // beginning of turn - check if tilt is outisde zero-thresh range
            // if(!returning && (abs( curr_x_val - sampled_accel_x ) > ACCEL_OUT_THRESH) ){
            if(!returning && (abs( curr_x_val - sampled_accel_x ) > ACCEL_OUT_THRESH) ){
                if( 
                (leftSignal && (curr_x_val < sampled_accel_x) ) ||
                ( !leftSignal && (curr_x_val > sampled_accel_x) ) ){
                    led_on(LED_2);
                    thresh_out_count++;  
                } 

                thresh_out_count++;
                if(thresh_out_count > ACCEL_TILT_THRESH){
                    returning = true;
                    thresh_out_count = 0;
                    led_off(LED_2);

                }
            }


            //check for return into zero-thresh
            if(returning && (abs( curr_x_val - sampled_accel_x ) <= ACCEL_IN_THRESH) ){
                if( 
                (leftSignal && (curr_x_val < sampled_accel_x) ) ||
                ( !leftSignal && (curr_x_val > sampled_accel_x) ) ){ 
                    led_on(LED_1);
                    thresh_in_count++;
                } 

                if( thresh_in_count > ACCEL_TILT_THRESH){
                    returning = false;
                    thresh_in_count = 0;
                    checkForReturn = false;
                    led_off(LED_0);
                    led_off(LED_1);
                }
            }

            // reset accelDataReady
            accelDataReady = false;
            
        }
        // reset the data ready interrupt by reading an axis reg
        readAxisX();









            // while(check_accel_x() == false){
            //     led_toggle(LED_1);
            //     for(time_cnt = 0; time_cnt < 1000000; time_cnt++){
            //     //do nothing, jsut to delay
            //     }
            // }
            // if(check_accel_x()){
                // checkForReturn = false;

            // }
        // }
        // if (b22 == true) {
        //     b22 = false;
        //     led_toggle(LED_1);
        // }

        // if (b8 == true) {
        //     b8 = false;
        //     led_toggle(LED_2);
        // }
        // if (b9 == true) {
        //     b9 = false;
        //     for (i=0;i<1000;++i) {
        //         if (i%100 == 0) {
        //             led_toggle(LED_0); 
        //         }
        //     }
        //     led_off(LED_0); 
        // }
        // if (b10 == true) {
        //     b9 = false;
        //     for (i=0;i<1000;++i) {
        //         if (i%100 == 0) {
        //             led_toggle(LED_1); 
        //         }
        //     }
        //     led_off(LED_1); 
        // }

    }
}


