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
#include "AccelDataDriver.h"
#include "AccelTurnControl.h"
#include "LightControl.h"
#include "LightAction.h"

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

/************* STATE MACHINE *************/
// // state = *(Side, R/L) *(Stable/Thresh) *(Tilt, No Tilt)
// enum TS_state {
//     RESET,  // 
//     RSN,    // Right - Stable - No Tilt
//     RTT,    // Right - Thresh - Tilt
//     RST,    // Right - Stable - Tilt
//     RTN,    // Right - Thresh - No Tilt
// 	LSN,    // Left  - Stable - No Tilt
//     LTT,    // Left  - Thresh - Tilt
//     LST,    // Left  - Stable - Tilt
//     LTN     // Left  - Tilt   - No Tilt
// };

#define BUTTON_PIN 21

#define BUTTON2_PIN 22

#define PIN1 8 
#define PIN2 9
#define ACCEL_PIN 10
static volatile uint32_t i = 0;
static bool accelDataReady = false;
//volatile int16_t sampled_accel_x = 0;
// static const int16_t sampled_accel_x = 0;
volatile int16_t curr_x_total = 0;
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
volatile bool checkForReturn, returning;

// struct LightAction {
//     LightState action; 
//     LightType pos;
// };

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
    }    
}

// checks if the accelerometer is within the threshold of the sampled value
//ASSUME THAT RIGHT TILT IS POSITIVE AND LEFT IS NEGATIVE
// bool check_passed_thresh(int16_t x_val, bool IO_thresh, bool LR_tilt){
//     int16_t thresh = (LH_thresh ? ACCEL_OUT_THRESH : ACCEL_IN_THRESH);
//     if(LR_tilt) { //Right tilt
//         return ( x_val - sampled_accel_x >= thresh );
//     } else { //Left tilt
//         return ( x_val - sampled_accel_x <= thresh * -1 );
//     }
// }

// LightAction* turn_signal_check(bool LB, 
//                        bool RB, 
//                        bool newAccelData, 
//                        int16_t accel_x_val){

//     static TS_state ts_state = RESET;

//     //Check if any state transitions will happen
//     if( ts_state == RESET ) {
//         if( !(RB || LB) ) {
//             return NULL;
//         }
//     } else {
//         if( !(RB || LB || newAccelData) ) {
//             return NULL;
//         }
//     }
    
//     return turn_signal_update(LB, RB, newAccelData, accel_x_val);
// }

// #define TS_RIGHT true
// #define TS_LEFT false
// #define TS_OUT true
// #define TS_IN false
// LightAction* turn_signal_update(bool LB, 
//                        bool RB, 
//                        bool newAccelData, 
//                        int16_t accel_x_val){
//     static int16_t count = 0;

//     //Transition State
//     switch(ts_state) {
//         case RESET: 
//         {
//             count = 0;
//             if (LB) {
//                 ts_state = LSN;
//             }
//             else if (RB) {
//                 ts_state = RSN;
//             }
//             break;
//         }
//         case RSN:
//         {
//             count = 0;
//             if(LB) {
//                 ts_state = LSN;
//             } 
//             else if(RB) {
//                 ts_state = RESET;        
//             }
//             else if( newAccelVal && 
//                      check_passed_thresh(accel_x_val, TS_OUT, TS_RIGHT)  ){
//                 ts_state = RTT;
//             }
//             break;
//         }
//         case RTT:
//         {
//             if(LB) {
//                 ts_state = LSN;
//             } 
//             else if (RB) {
//                 ts_state = RESET;
//             }
//             else if (newAccelVal) {
//                 if( check_passed_thresh(accel_x_val, TS_OUT, TS_RIGHT) ){
//                     count++;
//                 } else {
//                     ts_state = RSN;
//                 }
//             }
            
//             if (count > ACCEL_TILT_THRESH) {
//                 ts_state = RST;
//             }
//             break;
//         }
//         case RST:
//         {
//             count = 0;
//             if(LB) {
//                 ts_state = LSN;
//             } 
//             else if(RB) {
//                 ts_state = RESET;        
//             }
//             else if(newAccelVal &&
//                     !check_passed_thresh(accel_x_val, TS_IN, TS_RIGHT) ){
//                 ts_state = RTN;
//             }
//             break;
//         }
//         case RTN:
//         {
//             if(LB) {
//                 ts_state = LSN;
//             } 
//             else if (RB) {
//                 ts_state = RESET;
//             }
//             else if (newAccelVal) {
//                 if( !check_passed_thresh(accel_x_val, TS_IN, TS_RIGHT) ){
//                     count++;
//                 } else {
//                     ts_state = RST;
//                 }
//             }
            
//             if (count > ACCEL_TILT_THRESH) {
//                 ts_state = RESET;
//             }
//             break;
//         }
//         case LSN:
//         {
//             count = 0;
//             if(RB) {
//                 ts_state = RSN;
//             } 
//             else if (LB) {
//                 ts_state = RESET;
//             }
//             else if(newAccelVal &&
//                     check_passed_thresh(accel_x_val, TS_OUT, TS_LEFT) ){
//                 ts_state = LTT;
//             }
//             break;
//         }
//         case LTT:
//         {
//             if(RB) {
//                 ts_state = RSN;
//             } 
//             else if (LB) {
//                 ts_state = RESET;
//             }
//             else if (newAccelVal) {
//                 if( check_passed_thresh(accel_x_val, TS_OUT, TS_LEFT) ){
//                     count++;
//                 } else {
//                     ts_state = LSN;
//                 }
//             }

//             if (count > ACCEL_TILT_THRESH) {
//                 ts_state = LST;
//             }
//             break;
//         }
//         case LST:
//         {
//             count = 0;
//             if(RB) {
//                 ts_state = RSN;
//             } 
//             else if (LB) {
//                 ts_state = RESET;
//             }
//             else if(newAccelVal &&
//                     !check_passed_thresh(accel_x_val, TS_IN, TS_LEFT) ){
//                 ts_state = LTN;
//             }

//             break;
//         }
//         case LTN:
//         {
//             if(RB) {
//                 ts_state = RSN;
//             } 
//             else if (LB) {
//                 ts_state = RESET;
//             }
//             else if (newAccelVal) {
//                 if( !check_passed_thresh(accel_x_val, TS_IN, TS_LEFT) ){
//                     count++;
//                 } else {
//                     ts_state = LST;
//                 }
//             }
            
//             if (count > ACCEL_TILT_THRESH) {
//                 ts_state = RESET;
//             }
//             break;
//         }
//         default:
//             break;
//     }


//     //OUTPUT

//     switch(ts_state) {
//         case RESET: 
//             return  
//         case RSN:
//         case RTT:
//         case RST:
//         case RTN:
//         case LSN:
//         case LTT:
//         case LST:
//         case LTN:
//         default:
//             break;
//     }
// }

int main(void) {
    // LightAction light_act;

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
    // led_on(LED_2);

    //initializeAccelerometer();
    initializeAccelerometer();

    // since active high, pins need to be set to have a pull-down resistor,
    //      otherwise they will be floating
    static gpio_cfg_t cfgs[] = {  {9, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler, PIN_GPIOTE_IN},
                                     {10, GPIO_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, &pin_handler, PIN_GPIOTE_IN},
                                     {ACCEL_PIN, GPIO_ACTIVE_HIGH, NRF_GPIO_PIN_NOPULL, &pin_handler, PIN_GPIOTE_IN}};
    // // It seems only 4 pins can be registered per channel
    gpio_input_count = 2;

    /* SET INPUT WITH DRIVER */
    err_code = gpio_init(cfgs, gpio_input_count);
    if (err_code) {
        led_on(LED_1);
    }
    gpio_input_enable_all();

    //NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled; //Set GPIOTE interrupt register on channel 0
    //NVIC_EnableIRQ(GPIOTE_IRQn); //Enable interrupts

    setPollAccelData(DATA_X);

    //accelDataReady = true;
    // reset accelerometer data ready interrupt
    readAxisX();
    while (true) {

        //Get New Values
        // int16_t curr_x_val = 0;

        //if(accelDataReady) {
          populateAccelDataBank();
          //accelDataReady = false;
        //}
        bool newAccelVal = grabAccelData(DATA_X, &curr_x_val, NULL);


        // bool RB = false, LB = false;

        btn_state_change(b21, b22);
        if(b21 == true) {
          //Effectively latch value of b21
          // LB = true;
          b21 = false;
          
        }

        if(b22 == true) {
          //Effectively latch the value of b22
          // RB = true;
          b22 = false;
        }

        if(newAccelVal){
            light_act = do_state_action( curr_x_val );
        }

        if(light_act == LIGHT_ACTION_LEFT_TURN){
            led_on(LED_0);
            led_off(LED_1);
        }else if(light_act == LIGHT_ACTION_RIGHT_TURN){
            led_on(LED_1);
            led_off(LED_0);
        }else{
            led_off(LED_0);
            led_off(LED_1);
        }


        // if(newAccelVal){
            // led_toggle(LED_2);
        // }else{
            // led_off(LED_2);
        // }
    // readAxisX();
        // if(readAxisX() > 0){
        //     led_on(LED_0);
        // }else{
        //     led_off(LED_0);
        // }

        // if(readAxisZ() > 500){
        //     led_on(LED_1);
        // }else{
        //     led_off(LED_1);
        // }
    }

//////////////////////////////////////////////////
        // if (b21 == true) { //button was toggled ,left turn
        //     b21 = false;

        //     // check for anything on & turn it off
        //     if (checkForReturn && leftSignal) {
        //         checkForReturn = false;
        //         led_off(LED_0);
        //     } else if(checkForReturn && !leftSignal){ // right turn pressed
        //         // turn off everything for right
        //         led_off(LED_1);
        //         led_on(LED_0);
        //         leftSignal = true; // now processing left turn
        //     } else { // new press with blank slate
        //         leftSignal = true;
        //         checkForReturn = true;
        //         led_on(LED_0);
        //     }

        //     // reset counts
        //     thresh_out_count    = 0;
        //     thresh_in_count     = 0;
        //     led_off(LED_2);
        // }

        // if (b22 == true) { //button was toggled, right turn
        //     b22 = false;
            
        //     // check for anything on & turn it off
        //     if (checkForReturn && !leftSignal) { // right turn toggle off
        //         checkForReturn = false;
        //         led_off(LED_1);
        //     } else if(checkForReturn && leftSignal){ // left turn pressed
        //         // turn off everything for left
        //         led_off(LED_0);
        //         led_on(LED_1);
        //         leftSignal = false; // now processing right turn
        //     } else { // new press with blank slate
        //         leftSignal = false;
        //         checkForReturn = true;
        //         led_on(LED_1);
        //     }

        //     // reset counts
        //     thresh_out_count    = 0;
        //     thresh_in_count     = 0;
        //     led_off(LED_2);
        // }

        // if(checkForReturn && accelDataReady){
        //     populateAccelDataBank();
        //     int16_t curr_x_val;
        //     if (grabAccelData(DATA_X, &curr_x_val, NULL)) {
        //         // beginning of turn - check if tilt is outisde zero-thresh range
        //         if(!returning && (abs( curr_x_val - sampled_accel_x ) > ACCEL_OUT_THRESH) ){
        //             if( 
        //             (leftSignal && (curr_x_val < sampled_accel_x) ) ||
        //             ( !leftSignal && (curr_x_val > sampled_accel_x) ) ){
        //                 led_on(LED_2);
        //                 thresh_out_count++;  
        //             }                     

        //             // led_on(LED_2);
        //             // thresh_out_count++;
        //             if(thresh_out_count > ACCEL_TILT_THRESH){
        //                 returning = true;
        //                 thresh_out_count = 0;
        //                 led_off(LED_2);

        //             }
        //         }


        //         //check for return into zero-thresh
        //         if(returning && (abs( curr_x_val - sampled_accel_x ) <= ACCEL_IN_THRESH) ){
        //             led_on(LED_1);
        //             thresh_in_count++;

        //             if( thresh_in_count > ACCEL_TILT_THRESH){
        //                 returning = false;
        //                 thresh_in_count = 0;
        //                 checkForReturn = false;
        //                 led_off(LED_0);
        //                 led_off(LED_1);
        //             }
        //         }
        //     }

        //     // reset accelDataReady
        //     accelDataReady = false;

        // }
        // reset the data ready interrupt by reading an axis reg
    //     readAxisX();
    // }
}

